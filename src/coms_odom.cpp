#include <coms_odom/coms_odom.h>

/* Constructors, Destructor, and Assignment operators {{{ */
// Default constructor
ComsOdom::ComsOdom(unsigned int counts_per_rotation,
                   double encoder_to_axis_center,
                   double wheel_diameter,
                   const std::string& odom_frame,
                   const std::string& base_frame)
    : counts_per_rotation{counts_per_rotation}
    , encoder_to_axis_center{encoder_to_axis_center}
    , wheel_diameter{wheel_diameter}
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

// Copy constructor
ComsOdom::ComsOdom(const ComsOdom& other)
    : nh{other.nh}
    , odom_pub{other.odom_pub}
    , odom_broadcaster{other.odom_broadcaster}
    , counts_per_rotation{other.counts_per_rotation}
    , encoder_to_axis_center{other.encoder_to_axis_center}
    , wheel_diameter{other.wheel_diameter}
    , odom_frame{other.odom_frame}
    , base_frame{other.base_frame}
    , xyz{other.xyz}
    , rpy{other.rpy}
    , d_xyz{other.d_xyz}
    , d_rpy{other.d_rpy}
    , offset{other.offset}
    , speed{other.speed}
    , last_encoder_count{other.last_encoder_count}
    , last_encoder_time{other.last_encoder_time}
    , is_first_encoder_msg{other.is_first_encoder_msg}
    , current_imu_data{other.current_imu_data}
    , last_imu_data{other.last_imu_data}
    , is_first_imu_msg{other.is_first_imu_msg}
{
}

// Move constructor
ComsOdom::ComsOdom(ComsOdom&& other)
    : nh{std::move(other.nh)}
    , odom_pub{std::move(other.odom_pub)}
    , odom_broadcaster{std::move(other.odom_broadcaster)}
    , counts_per_rotation{std::move(other.counts_per_rotation)}
    , encoder_to_axis_center{std::move(other.encoder_to_axis_center)}
    , wheel_diameter{std::move(other.wheel_diameter)}
    , odom_frame{std::move(other.odom_frame)}
    , base_frame{std::move(other.base_frame)}
    , xyz{std::move(other.xyz)}
    , rpy{std::move(other.rpy)}
    , d_xyz{std::move(other.d_xyz)}
    , d_rpy{std::move(other.d_rpy)}
    , offset{std::move(other.offset)}
    , speed{std::move(other.speed)}
    , last_encoder_count{std::move(other.last_encoder_count)}
    , last_encoder_time{std::move(other.last_encoder_time)}
{
}

// Destructor
ComsOdom::~ComsOdom()
{
}

// Assignment operator
ComsOdom&
ComsOdom::operator=(const ComsOdom& other) {
    nh = other.nh;
    odom_pub = other.odom_pub;
    odom_broadcaster = other.odom_broadcaster;
    counts_per_rotation = other.counts_per_rotation;
    encoder_to_axis_center = other.encoder_to_axis_center;
    wheel_diameter = other.wheel_diameter;
    odom_frame = other.odom_frame;
    base_frame = other.base_frame;
    xyz = other.xyz;
    rpy = other.rpy;
    d_xyz = other.d_xyz;
    d_rpy = other.d_rpy;
    offset = other.offset;
    speed = other.speed;
    last_encoder_count = other.last_encoder_count;
    last_encoder_time = other.last_encoder_time;
    is_first_encoder_msg = other.is_first_encoder_msg;
    current_imu_data = other.current_imu_data;
    last_imu_data = other.last_imu_data;
    is_first_imu_msg = other.is_first_imu_msg;
    return *this;
}

// Move assignment operator
ComsOdom&
ComsOdom::operator=(ComsOdom&& other) {
    nh = std::move(other.nh);
    odom_pub = std::move(other.odom_pub);
    odom_broadcaster = std::move(other.odom_broadcaster);
    counts_per_rotation = std::move(other.counts_per_rotation);
    encoder_to_axis_center = std::move(other.encoder_to_axis_center);
    wheel_diameter = std::move(other.wheel_diameter);
    odom_frame = std::move(other.odom_frame);
    base_frame = std::move(other.base_frame);
    xyz = std::move(other.xyz);
    rpy = std::move(other.rpy);
    d_xyz = std::move(other.d_xyz);
    d_rpy = std::move(other.d_rpy);
    offset = std::move(other.offset);
    speed = std::move(other.speed);
    last_encoder_count = std::move(other.last_encoder_count);
    last_encoder_time = std::move(other.last_encoder_time);
    is_first_encoder_msg = std::move(other.is_first_encoder_msg);
    current_imu_data = std::move(other.current_imu_data);
    last_imu_data = std::move(other.last_imu_data);
    is_first_imu_msg = std::move(other.is_first_imu_msg);
    return *this;
}
/* }}} */

void
ComsOdom::encoder_callback(const std_msgs::UInt32& data) {
    auto current_count = data.data;
    // TODO: use stamped message for encoder
    auto current_time = ros::Time::now();

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
    d_rpy = std::make_tuple(current_imu_data.angular_velocity.x,
                            -current_imu_data.angular_velocity.y,
                            -current_imu_data.angular_velocity.z);
    auto imu_dt = (current_imu_data.header.stamp
                   - last_imu_data.header.stamp).toSec();

    rpy = std::make_tuple(
        limit_rad(std::get<0>(rpy) + (std::get<0>(d_rpy) * imu_dt)),
        limit_rad(std::get<1>(rpy) + (std::get<1>(d_rpy) * imu_dt)),
        limit_rad(std::get<2>(rpy) + (std::get<2>(d_rpy) * imu_dt))
    );

    auto yaw = std::get<2>(rpy);

    // Then, calculate the position
    // TODO: Consider roll, pitch, and yaw
    xyz = std::make_tuple(
        std::get<0>(xyz) + (offset * std::cos(yaw)),
        std::get<1>(xyz) + (offset * std::sin(yaw)),
        0
    );
    // TODO: encoder_to_axis_center is probably going to be used here
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
    odom.twist.twist.angular.x = std::get<0>(d_rpy);
    odom.twist.twist.angular.y = std::get<1>(d_rpy);
    odom.twist.twist.angular.z = std::get<2>(d_rpy);
    odom_pub.publish(odom);
}