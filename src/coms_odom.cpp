#include <coms_odom/coms_odom.h>

/* Constructors, Destructor, and Assignment operators {{{ */
// Default constructor
ComsOdom::ComsOdom(unsigned int counts_per_rotation,
                   double track,
                   double wheel_diameter,
                   unsigned int drift_correction,
                   const std::string& odom_frame,
                   const std::string& base_frame)
    : counts_per_rotation{counts_per_rotation}
    , track{track}
    , wheel_diameter{wheel_diameter}
    , drift_correction{drift_correction}
    , past_imu{drift_correction}
    , last_calculated_drift{0}
    , odom_frame{odom_frame}
    , base_frame{base_frame}
    , is_first_encoder_msg{true}
    , is_first_imu_msg{true}
{
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);

    xyz = std::make_tuple(0, 0, 0);
    rpy = std::make_tuple(0, 0, 0);
    d_xyz = std::make_tuple(0, 0, 0);
    d_rpy = std::make_tuple(0, 0, 0);
}
/* }}} */

void
ComsOdom::encoder_callback(const coms_msgs::ComsEncoder& data) {
    auto current_time = data.header.stamp;
    auto current_count = data.count;

    if (is_first_encoder_msg) {
        last_encoder_count = current_count;
        last_encoder_time = current_time;
        is_first_encoder_msg = false;
        return;
    }

    offset = M_PI * wheel_diameter
             * static_cast<int>(current_count - last_encoder_count)
             / counts_per_rotation;
    auto dt = current_time - last_encoder_time;
    speed = offset / dt.toSec();

    last_encoder_count = current_count;
    last_encoder_time = current_time;
}

void
ComsOdom::imu_callback(const sensor_msgs::Imu& data) {
    last_imu_data = std::move(current_imu_data);
    current_imu_data = data;

    if (!is_first_imu_msg) {
        send_transforms();
    }

    is_first_imu_msg = false;

    // Can't calculate drift when moving
    if (speed == 0) {
        past_imu.push_back(current_imu_data);
    }
    else {
        past_imu.clear();
    }
}

void
ComsOdom::send_transforms() {
    auto t = current_imu_data.header.stamp;

    // First, calculate the orientation of the vehicle
    /*
     * VG440 is fixed on the COMS in the following orientation:
     * x: points to the front of the vehicle
     * y: points to the right of the vehicle
     * z: points to the bottom of the vehicle
     *
     * On the other hand, ROS's convention is:
     * x: points to the front of the vehicle
     * y: points to the left of the vehicle
     * z: points to the top of the vehicle
     *
     * We must rotate the axes accordingly.
     */
    d_rpy = std::make_tuple(
        current_imu_data.angular_velocity.x,
        -current_imu_data.angular_velocity.y,
        -(current_imu_data.angular_velocity.z - drift())
    );
    /*
     * Get the speed at the center of body
     */
    speed += std::get<2>(d_rpy) * track / 2;

    rpy = get_rpy();
    auto yaw = std::get<2>(rpy);

    // Then, calculate the position
    // TODO: Consider roll, pitch, and yaw
    xyz = std::make_tuple(
        std::get<0>(xyz) + (offset * std::cos(yaw)),
        std::get<1>(xyz) + (offset * std::sin(yaw)),
        0
    );
    d_xyz = std::make_tuple(
        speed * std::cos(yaw),
        speed * std::sin(yaw),
        0
    );

    // See the following URL for details:
    // http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot
    // TODO: only considers x, y, theta (3 DOF). Change to 6 DOF
    geometry_msgs::TransformStamped odom_to_base_link;
    odom_to_base_link.header.stamp = t;
    odom_to_base_link.header.frame_id = odom_frame;
    odom_to_base_link.child_frame_id = base_frame;
    odom_to_base_link.transform.translation.x = std::get<0>(xyz);
    odom_to_base_link.transform.translation.y = std::get<1>(xyz);
    odom_to_base_link.transform.translation.z = std::get<2>(xyz);
    tf2::Quaternion rpy_quat;
    rpy_quat.setRPY(std::get<0>(rpy), std::get<1>(rpy), std::get<2>(rpy));
    odom_to_base_link.transform.rotation = tf2::toMsg(rpy_quat);
    odom_broadcaster.sendTransform(odom_to_base_link);

    /*
    // TODO: odom to base_footprint
    geometry_msgs::TransformStamped odom_to_base_footprint;
    odom_to_base_footprint.header.stamp = t;
    odom_to_base_footprint.header.frame_id = odom_frame;
    odom_to_base_footprint.child_frame_id = "base_footprint";
    odom_broadcaster.sendTransform(odom_to_base_footprint);

    // TODO: base_footprint to base_stabilized
    geometry_msgs::TransformStamped base_footprint_to_stabilized;
    base_footprint_to_stabilized.header.frame_id = "base_footprint";
    base_footprint_to_stabilized.child_frame_id = "base_stabilized";
    odom_broadcaster.sendTransform(base_footprint_to_stabilized);

    // TODO: base_stabilized to base_link
    geometry_msgs::TransformStamped base_stabilized_to_link;
    base_stabilized_to_link.header.frame_id = "base_stabilized";
    base_stabilized_to_link.child_frame_id = base_frame;
    odom_broadcaster.sendTransform(base_stabilized_to_link);
    */

    // Publish /odom topic
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame;
    // Position
    odom.pose.pose.position.x = std::get<0>(xyz);
    odom.pose.pose.position.y = std::get<1>(xyz);
    odom.pose.pose.position.z = std::get<2>(xyz);
    odom.pose.pose.orientation = tf2::toMsg(rpy_quat);
    // Velocity
    odom.child_frame_id = base_frame;
    odom.twist.twist.linear.x = std::get<0>(d_xyz);
    odom.twist.twist.linear.y = std::get<1>(d_xyz);
    odom.twist.twist.linear.z = std::get<2>(d_xyz);
    // Angular velocity
    odom.twist.twist.angular.x = std::get<0>(d_rpy);
    odom.twist.twist.angular.y = std::get<1>(d_rpy);
    odom.twist.twist.angular.z = std::get<2>(d_rpy);
    odom_pub.publish(odom);
}

double
ComsOdom::limit_rad(const double val) {
    if (val > M_PI) {
        return val - (2 * M_PI);
    }
    else if (val < -M_PI) {
        return val + (2 * M_PI);
    }
    return val;
}

std::tuple<double, double, double>
ComsOdom::get_rpy() {
    auto imu_dt = (current_imu_data.header.stamp
                   - last_imu_data.header.stamp).toSec();
    auto yaw = limit_rad(std::get<2>(rpy) + (std::get<2>(d_rpy) * imu_dt));

    if (drift_correction == 0) {
        return std::make_tuple(0, 0, yaw);
    }

    return std::make_tuple(0, 0, yaw - drift());
}

double
ComsOdom::drift() {
    if (past_imu.empty()) {
        return last_calculated_drift;
    }

    // Calculate average
    double omega_sum = 0;
    for (const auto& imu : past_imu) {
        omega_sum += imu.angular_velocity.z;
    }
    last_calculated_drift = omega_sum / past_imu.size();

    return last_calculated_drift;
}

double
ComsOdom::q_to_yaw(const geometry_msgs::Quaternion& q) {
    double r, p, y;
    std::tie(r, p, y) = q_to_rpy(q);
    return y;
}

std::tuple<double, double, double>
ComsOdom::q_to_rpy(const geometry_msgs::Quaternion& q) {
    auto tf_q = tf2::Quaternion{q.x, q.y, q.z, q.w};
    double r, p, y;
    tf2::Matrix3x3{tf_q}.getRPY(r, p, y);
    return std::make_tuple(r, p, y);
}
