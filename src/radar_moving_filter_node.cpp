#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/filters/filter.h> 
#include "pcl/filters/impl/filter.hpp"
#include "detection/radar_type.hpp"
#include "detection/fusion_function.hpp"
#include "a2rl_bs_msgs/msg/vectornav_ins.hpp"

#define LIDAR_FILTER_ANGLE 52


Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d R = q.matrix();
    return R;
}

Eigen::Matrix4d createTransformationMatrix(Eigen::Vector3d &rpy, Eigen::Vector3d &xyz) {
    Eigen::Matrix3d rotationMatrix = rpyToRotationMatrix(rpy(0), rpy(1), rpy(2));
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix.block<3, 1>(0, 3) = xyz;
    return transformationMatrix;
}

void confidence_filter(pcl::PointCloud<RadarPointType>::Ptr cloud_in, pcl::PointCloud<RadarPointType>::Ptr cloud_out){
    for (const auto& p : *cloud_in) {
        RadarPointType point;
        float range = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (range < 1.0 || range > 350.0 || p.x * p.x < 1e-5 || p.z < -1.0 || p.confidence < 0.3){
            continue;
        }
        double offset = 0.0;
        if (range < 100.0)
        {
            offset = 77.0/0.86 * 0.00019;
        }
        else if (range > 100.0  && range < 200.0)
        {
            offset = 77.0/0.45 * 0.00019;
        }
        else if(range > 200.0  && range < 350.0 )
        {
            offset = 77.0/0.26 * 0.00019;
        }
        // ROS_INFO("offset is: %f", offset * p.velocity);
        point.x = p.x - offset * p.velocity;
        point.y = p.y - offset * p.velocity;
        point.z = p.z - offset * p.velocity;
        point.velocity = p.velocity;
        point.snr = p.snr;
        point.rcs = p.rcs;
        point.confidence = p.confidence;
        point.velocity_interval = p.velocity_interval;

        cloud_out->push_back(point);
    }
}

class RadarFusionNode : public rclcpp::Node
{
public:
    RadarFusionNode() : Node("radar_moving_node")
    {
        subscription_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/radar_front/points", 10, std::bind(&RadarFusionNode::front_pointsCallback, this, std::placeholders::_1));
        subscription_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/radar_left/points", 10, std::bind(&RadarFusionNode::left_pointsCallback, this, std::placeholders::_1));
        subscription_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/radar_right/points", 10, std::bind(&RadarFusionNode::right_pointsCallback, this, std::placeholders::_1));
        subscription_back_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/radar_back/points", 10, std::bind(&RadarFusionNode::back_pointsCallback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dynamic_radar_points", 10);

        vectornav_subscriber_ = this->create_subscription<a2rl_bs_msgs::msg::VectornavIns>("/a2rl/vn/ins", 10, std::bind(&MultiVehicleDetection::vectornav_callback, this, std::placeholders::_1));
    }

private:
    void vectornav_callback(const a2rl_bs_msgs::msg::VectornavIns::SharedPtr msg) {
        latest_vn_msg_ = *msg;
    }
    void front_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_front) {
            front_cloud->clear();
        }
        pcl::PointCloud<RadarPointType>::Ptr cloud(new pcl::PointCloud<RadarPointType>);
        pcl::fromROSMsg(*msg, *cloud);
        pcl::PointCloud<RadarPointType>::Ptr cloud_filtered(new pcl::PointCloud<RadarPointType>);
        confidence_filter(cloud,cloud_filtered);
        VAP(cloud_filtered, front_cloud, latest_vn_msg_.velocity_body_ins.x, latest_vn_msg_.velocity_body_ins.y);

        count_front = true;
        if(count_front && count_left && count_right && count_back){
            publish(msg->header.stamp);
        }
    }
    void left_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_left) {
            left_cloud->clear();
        }
        pcl::PointCloud<RadarPointType>::Ptr cloud(new pcl::PointCloud<RadarPointType>);
        pcl::fromROSMsg(*msg, *cloud);
        pcl::PointCloud<RadarPointType>::Ptr cloud_filtered(new pcl::PointCloud<RadarPointType>);
        confidence_filter(cloud,cloud_filtered);
        VAP(cloud_filtered, left_cloud, latest_vn_msg_.velocity_body_ins.x, latest_vn_msg_.velocity_body_ins.y);

        count_left = true;
        if(count_front && count_left && count_right && count_back){
            publish(msg->header.stamp);
        }
    }
    void right_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_right) {
            right_cloud->clear();
        }
        pcl::PointCloud<RadarPointType>::Ptr cloud(new pcl::PointCloud<RadarPointType>);
        pcl::fromROSMsg(*msg, *cloud);
        pcl::PointCloud<RadarPointType>::Ptr cloud_filtered(new pcl::PointCloud<RadarPointType>);
        confidence_filter(cloud,cloud_filtered);
        VAP(cloud_filtered, right_cloud, latest_vn_msg_.velocity_body_ins.x, latest_vn_msg_.velocity_body_ins.y);

        count_right = true;
        if(count_front && count_left && count_right && count_back){
            publish(msg->header.stamp);
        }
    }
    void back_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_back) {
            back_cloud->clear();
        }
        pcl::PointCloud<RadarPointType>::Ptr cloud(new pcl::PointCloud<RadarPointType>);
        pcl::fromROSMsg(*msg, *cloud);
        pcl::PointCloud<RadarPointType>::Ptr cloud_filtered(new pcl::PointCloud<RadarPointType>);
        confidence_filter(cloud,cloud_filtered);
        VAP(cloud_filtered, back_cloud, latest_vn_msg_.velocity_body_ins.x, latest_vn_msg_.velocity_body_ins.y);

        count_back = true;
        if(count_front && count_left && count_right && count_back){
            publish(msg->header.stamp);
        }
    }
    void publish(builtin_interfaces::msg::Time time){
        pcl::PointCloud<RadarPointType>::Ptr merged_cloud_(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr f_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr l_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr r_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr b_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr merged_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        if(right_cloud->size()>0){
            pcl::transformPointCloud(*right_cloud, *r_transformed_cloud, T_right);
            *merged_cloud_ += *r_transformed_cloud;
        }
        if(left_cloud->size()>0){
            pcl::transformPointCloud(*left_cloud, *l_transformed_cloud, T_left);
            *merged_cloud_ += *l_transformed_cloud;
        }
        if(front_cloud->size()>0){
            pcl::transformPointCloud(*front_cloud, *f_transformed_cloud, T_front);
            *merged_cloud_ += *f_transformed_cloud;
        }
        if(back_cloud->size()>0){
            pcl::transformPointCloud(*back_cloud, *b_transformed_cloud, T_back);
            *merged_cloud_ += *b_transformed_cloud;
        }
        if(merged_cloud_->size()>0){
            pcl::transformPointCloud(*merged_cloud_, *merged_transformed_cloud, T_body);
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*merged_transformed_cloud, output);
        output.header.stamp = time;
        RCLCPP_INFO(this->get_logger(),"Current time: %ld seconds and %ld nanoseconds", 
            output.header.stamp.sec, 
            output.header.stamp.nanosec % 1000000000);
        output.header.frame_id = "map";
        publisher_->publish(output);
        front_cloud->clear();
        left_cloud->clear();
        right_cloud->clear();
        back_cloud->clear();
        count_front = false;
        count_left = false;
        count_right = false;
        count_back = false;
    }

    //Settings from urdf
    Eigen::Vector3d rpy_body = Eigen::Vector3d(3.14, -1.62, 0);
    Eigen::Vector3d xyz_body = Eigen::Vector3d(0.63, 0, 0.61);
    Eigen::Vector3d rpy_front = Eigen::Vector3d(1.525, -0.000, 1.565);
    Eigen::Vector3d xyz_front = Eigen::Vector3d(0.077, 0.014, -0.240);
    Eigen::Vector3d rpy_left = Eigen::Vector3d(0.463, -1.552, 2.679);
    Eigen::Vector3d xyz_left = Eigen::Vector3d(-0.015, -0.066, -0.268);
    Eigen::Vector3d rpy_right = Eigen::Vector3d(-0.009, 1.550, -0.015);
    Eigen::Vector3d xyz_right = Eigen::Vector3d(0.003, 0.180, -0.290);
    Eigen::Vector3d rpy_back = Eigen::Vector3d(-1.562, 0.004, -1.566);
    Eigen::Vector3d xyz_back = Eigen::Vector3d(-0.315, 0.063, -3.069);
    bool count_left = false;
    bool count_right = false;
    bool count_front = false;
    bool count_back = false;
    Eigen::Matrix4d T_front = createTransformationMatrix(rpy_front,xyz_front);
    Eigen::Matrix4d T_left = createTransformationMatrix(rpy_left,xyz_left);
    Eigen::Matrix4d T_right = createTransformationMatrix(rpy_right,xyz_right);
    Eigen::Matrix4d T_back = createTransformationMatrix(rpy_back,xyz_back);
    Eigen::Matrix4d T_body = createTransformationMatrix(rpy_body,xyz_body);

    pcl::PointCloud<RadarPointType>::Ptr front_cloud{new pcl::PointCloud<RadarPointType>()};
    pcl::PointCloud<RadarPointType>::Ptr left_cloud{new pcl::PointCloud<RadarPointType>()};
    pcl::PointCloud<RadarPointType>::Ptr right_cloud{new pcl::PointCloud<RadarPointType>()};
    pcl::PointCloud<RadarPointType>::Ptr back_cloud{new pcl::PointCloud<RadarPointType>()};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_front_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_left_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_back_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    
    rclcpp::Subscription<a2rl_bs_msgs::msg::VectornavIns>::SharedPtr vectornav_subscriber_;
    a2rl_bs_msgs::msg::VectornavIns latest_vn_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
