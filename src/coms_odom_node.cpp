#include <coms_odom/coms_odom.h>

#include <ros/ros.h>

#include <string>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "coms_odom_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p{"~"};

    // Get parameters
    float wheel_diameter;
    int counts_per_rotation;
    float encoder_to_axis_center;
    std::string base_frame;
    std::string odom_frame;

    if (!nh_p.hasParam("wheel_diameter")) {
        ROS_ERROR("Required parameter wheel_diameter not specified");
        return 1;
    }
    if (!nh_p.hasParam("counts_per_rotation")) {
        ROS_ERROR("Required parameter counts_per_rotation not specified");
        return 1;
    }
    if (!nh_p.hasParam("encoder_to_axis_center")) {
        ROS_ERROR("Required parameter encoder_to_axis_center not specified");
        return 1;
    }

    nh_p.getParam("wheel_diameter", wheel_diameter);
    nh_p.getParam("counts_per_rotation", counts_per_rotation);
    nh_p.getParam("encoder_to_axis_center", encoder_to_axis_center);
    nh_p.param("base_frame", base_frame, std::string{"base_link"});
    nh_p.param("odom_frame", odom_frame, std::string{"odom"});

    ComsOdom coms_odom{static_cast<unsigned int>(counts_per_rotation),
                       encoder_to_axis_center,
                       wheel_diameter,
                       odom_frame,
                       base_frame};

    ros::Subscriber encoder_sub = nh.subscribe("encoder", 100, &ComsOdom::encoder_callback, &coms_odom);
    ros::Subscriber imu_sub = nh.subscribe("imu", 100, &ComsOdom::imu_callback, &coms_odom);

    ros::spin();

    return 0;
}
