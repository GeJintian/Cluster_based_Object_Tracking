#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "detection/detection.hpp"
#include <signal.h>
#include <pcl_conversions/pcl_conversions.h>

using PointType = pcl::PointXYZI;
using namespace std;

std::shared_ptr<MOT<PointType>> Detector;

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    cout << "Operating MOT..." << endl;
    Detector.reset(new MOT<PointType>(options));
    rclcpp::spin(Detector);
    rclcpp::shutdown();

    return 0;
}
