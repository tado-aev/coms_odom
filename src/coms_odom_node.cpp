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
    float track;
    std::string base_frame;
    std::string odom_frame;
    int drift_correction;
    double stop_threshold;

    if (!nh_p.hasParam("wheel_diameter")) {
        ROS_ERROR("Required parameter wheel_diameter not specified");
        return 1;
    }
    if (!nh_p.hasParam("counts_per_rotation")) {
        ROS_ERROR("Required parameter counts_per_rotation not specified");
        return 1;
    }
    if (!nh_p.hasParam("track")) {
        ROS_ERROR("Required parameter track not specified");
        return 1;
    }

    nh_p.getParam("wheel_diameter", wheel_diameter);
    nh_p.getParam("counts_per_rotation", counts_per_rotation);
    nh_p.getParam("track", track);
    nh_p.param("base_frame", base_frame, std::string{"base_link"});
    nh_p.param("odom_frame", odom_frame, std::string{"odom"});
    nh_p.param("drift_correction", drift_correction, 0);
    nh_p.param("stop_threshold", stop_threshold, 0.0);

    ComsOdom coms_odom{static_cast<unsigned int>(counts_per_rotation),
                       track,
                       wheel_diameter,
                       static_cast<unsigned int>(drift_correction),
                       stop_threshold,
                       odom_frame,
                       base_frame};

    ros::Subscriber encoder_sub = nh.subscribe("encoder", 100, &ComsOdom::encoder_callback, &coms_odom);
    ros::Subscriber imu_sub = nh.subscribe("imu", 100, &ComsOdom::imu_callback, &coms_odom);

    ros::spin();

    return 0;
}
