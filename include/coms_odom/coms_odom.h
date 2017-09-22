#ifndef COMS_ODOM_H_
#define COMS_ODOM_H_

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <coms_msgs/ComsEncoder.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>
#include <string>

class ComsOdom {
public:
    /* Constructors, Destructor, and Assignment operators {{{ */
    ComsOdom(unsigned int counts_per_rotation,
             double encoder_to_axis_center,
             double wheel_diameter,
             const std::string& odom_frame,
             const std::string& base_frame);

    // Copy constructor
    ComsOdom(const ComsOdom& other);

    // Move constructor
    ComsOdom(ComsOdom&& other);

    // Destructor
    ~ComsOdom();

    // Assignment operator
    ComsOdom&
    operator=(const ComsOdom& other);

    // Move assignment operator
    ComsOdom&
    operator=(ComsOdom&& other);
    /* }}} */

    void
    encoder_callback(const coms_msgs::ComsEncoder& data);

    void
    imu_callback(const sensor_msgs::Imu& data);

    void
    send_transforms();

private:
    ros::NodeHandle nh;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;

    /*
     * How many counts there are in one rotation.
     */
    unsigned int counts_per_rotation;
    /*
     * Distance, in meters, from the transducer to the center of the
     * wheel's axis. If the transducer is on the left wheel, value should
     * be negative.
     * TODO: This is currently not used. Use it!
     */
    double encoder_to_axis_center;
    /*
     * Diameter of the wheel
     */
    double wheel_diameter;

    std::string odom_frame;
    std::string base_frame;

    std::tuple<double, double, double> xyz, rpy;
    std::tuple<double, double, double> d_xyz, d_rpy;
    /* Distance offset and speed calculated from the encoder */
    double offset, speed;
    unsigned int last_encoder_count;
    ros::Time last_encoder_time;
    bool is_first_encoder_msg;

    sensor_msgs::Imu current_imu_data;
    sensor_msgs::Imu last_imu_data;
    bool is_first_imu_msg;

    /**
     * Limits the given radian to the range [-pi, pi]
     * @param val the radian value to be limited
     * @return The radian value in the range [-pi, pi]
     */
    double
    limit_rad(const double val);
};

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

#endif /* end of include guard */
