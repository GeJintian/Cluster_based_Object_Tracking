#ifndef DETECTION_H
#define DETECTION_H
#include "detection/fusion_function.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sstream>
#include <iostream>
//#include "visualizer.hpp"
#include "detection/fusion_function.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "a2rl_bs_msgs/msg/vectornav_ins.hpp"
#include "a2rl_bs_msgs/msg/perception.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>


using namespace std;

class MultiVehicleDetection : public rclcpp::Node
{
public:
    MultiVehicleDetection();
    ~MultiVehicleDetection();
    int count = 0;
private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vis_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vis_radar_publisher_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_cloud_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_radar_cloud_;
    std::shared_ptr<Sync> synchronizer_;
    rclcpp::Subscription<a2rl_bs_msgs::msg::VectornavIns>::SharedPtr vectornav_subscriber_;
    a2rl_bs_msgs::msg::VectornavIns latest_vn_msg_;
    bool is_frozen = false;

    void detection_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &lidar_msg, const sensor_msgs::msg::PointCloud2::ConstPtr &radar_msg);
    void vectornav_callback(const a2rl_bs_msgs::msg::VectornavIns::SharedPtr msg);
    void clustering_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar, pcl::PointCloud<RadarPointType>::Ptr cloud_radar, double init_x, double init_y);
    void topic1_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &lidar_msg);

    void topic2_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &lidar_msg);
};
#endif