#include <memory>

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "localization_wrapper.h"

//hayden： 主函数文件，实现初始化ros，初始化LocalizationWrapper类
int main (int argc, char** argv) {
    // Set glog.
    FLAGS_colorlogtostderr = true;

    // Initialize ros.
    ros::init(argc, argv, "imu_gps_localization");
    ros::NodeHandle nh;
    
    // Initialize localizer.
    LocalizationWrapper localizer(nh);

    ros::spin();
    return 1;
}