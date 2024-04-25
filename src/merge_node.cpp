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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/filters/filter.h>
#include "builtin_interfaces/msg/time.hpp"
#include "pcl/filters/impl/filter.hpp"

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


class LidarFusionNode : public rclcpp::Node
{
public:
    LidarFusionNode() : Node("lidar_fusion_node")
    {
        subscription_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_front/points", 10, std::bind(&LidarFusionNode::front_pointsCallback, this, std::placeholders::_1));
        subscription_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_left/points", 10, std::bind(&LidarFusionNode::left_pointsCallback, this, std::placeholders::_1));
        subscription_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_right/points", 10, std::bind(&LidarFusionNode::right_pointsCallback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10);
        RCLCPP_INFO(this->get_logger(), "Lidar Fusion Node has been started.");
    }

private:
    void front_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_front) {
            front_cloud->clear();
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *front_cloud);

        // for (const auto& point : *cloud) {
        //     double azimuth = std::atan2(point.y, point.z);

        //     double min_angle_rad = -LIDAR_FILTER_ANGLE * (M_PI / 180.0);
        //     double max_angle_rad = LIDAR_FILTER_ANGLE * (M_PI / 180.0);

        //     if (azimuth >= min_angle_rad && azimuth <= max_angle_rad) {
        //         front_cloud->push_back(point);
        //     }
        // }
        count_front = true;
        if(count_front && count_left && count_right){
            publish(msg->header.stamp);
        }
    }
    void left_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_left) {
            left_cloud->clear();
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        //pcl::fromROSMsg(*msg, *left_cloud);
        pcl::fromROSMsg(*msg, *left_cloud);

        // for (const auto& point : *cloud) {
        //     double azimuth = std::atan2(point.y, point.z);

        //     double min_angle_rad = -LIDAR_FILTER_ANGLE * (M_PI / 180.0);
        //     double max_angle_rad = LIDAR_FILTER_ANGLE * (M_PI / 180.0);

        //     if (azimuth >= min_angle_rad && azimuth <= max_angle_rad) {
        //         left_cloud->push_back(point);
        //     }
        // }
        count_left = true;
        if(count_front && count_left && count_right){
            publish(msg->header.stamp);
        }
    }
    void right_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_right) {
            right_cloud->clear();
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *right_cloud);

        // for (const auto& point : *cloud) {
        //     double azimuth = std::atan2(point.y, point.z);

        //     double min_angle_rad = -LIDAR_FILTER_ANGLE * (M_PI / 180.0);
        //     double max_angle_rad = LIDAR_FILTER_ANGLE * (M_PI / 180.0);

        //     if (azimuth >= min_angle_rad && azimuth <= max_angle_rad) {
        //         right_cloud->push_back(point);
        //     }
        // }
        count_right = true;
        if(count_front && count_left && count_right){
            publish(msg->header.stamp);
        }
    }
    void publish(builtin_interfaces::msg::Time time){
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud_(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr f_transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr l_transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr r_transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if(right_cloud->size()>0){
            pcl::transformPointCloud(*right_cloud, *r_transformed_cloud, T_right);
            *merged_cloud_ += *r_transformed_cloud;
        }
        if(left_cloud->size()>0){
        pcl::transformPointCloud(*left_cloud, *l_transformed_cloud, T_left);
        *merged_cloud_ += *l_transformed_cloud;}
        if(front_cloud->size()>0){
        pcl::transformPointCloud(*front_cloud, *f_transformed_cloud, T_front);
        *merged_cloud_ += *f_transformed_cloud;}

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*merged_cloud_, output);
        output.header.stamp = time;
        RCLCPP_INFO(this->get_logger(),"Current time: %d seconds and %d nanoseconds", 
            output.header.stamp.sec, 
            output.header.stamp.nanosec % 1000000000);
        output.header.frame_id = "map";
        publisher_->publish(output);
        front_cloud->clear();
        left_cloud->clear();
        right_cloud->clear();
        count_front = false;
        count_left = false;
        count_right = false;
    }

    // void publish(builtin_interfaces::msg::Time time){
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud_(new pcl::PointCloud<pcl::PointXYZI>());
    //     *merged_cloud_+= *right_cloud;
    //     *merged_cloud_+= *left_cloud;
    //     *merged_cloud_+= *front_cloud;
    //     sensor_msgs::msg::PointCloud2 output;
    //     pcl::toROSMsg(*merged_cloud_, output);
    //     output.header.stamp = time;
    //     RCLCPP_INFO(this->get_logger(),"Current time: %d seconds and %d nanoseconds", 
    //         output.header.stamp.sec, 
    //         output.header.stamp.nanosec % 1000000000);
    //     output.header.frame_id = "map";
    //     publisher_->publish(output);
    //     front_cloud->clear();
    //     left_cloud->clear();
    //     right_cloud->clear();
    //     count_front = false;
    //     count_left = false;
    //     count_right = false;
    // }

    //Settings from urdf
    Eigen::Vector3d rpy_front = Eigen::Vector3d(-0.002, -1.57, 3.142);
    Eigen::Vector3d xyz_front = Eigen::Vector3d(0.630, 0.000, 0.610);
    // Eigen::Vector3d xyz_front = Eigen::Vector3d(1.037, 0.000, 0.610);
    Eigen::Vector3d rpy_left = Eigen::Vector3d(-1.038, -1.54, -0.005);
    Eigen::Vector3d xyz_left = Eigen::Vector3d(-0.032, 0.191, 0.85); //-0.065 pitch
    // Eigen::Vector3d xyz_left = Eigen::Vector3d(1.697, 0.191, 0.648);
    Eigen::Vector3d rpy_right = Eigen::Vector3d(1.045, -1.55, -0.005);
    Eigen::Vector3d xyz_right = Eigen::Vector3d(-0.033, -0.189, 0.95);//-0.05 pitch
    // Eigen::Vector3d xyz_right = Eigen::Vector3d(1.697, -0.189, 0.648);
    // Eigen::Vector3d rpy_front = Eigen::Vector3d(3.14, -1.62, 0);
    // Eigen::Vector3d xyz_front = Eigen::Vector3d(0.63, 0, 0.61);
    // Eigen::Vector3d rpy_left = Eigen::Vector3d(2.10, -0.07, 0);
    // Eigen::Vector3d xyz_left = Eigen::Vector3d(0.071, -0.19, -0.66);
    // Eigen::Vector3d rpy_right = Eigen::Vector3d(-2.10, -0.07, 0);
    // Eigen::Vector3d xyz_right = Eigen::Vector3d(0.071, 0.19, -0.66);
    bool count_left = false;
    bool count_right = false;
    bool count_front = false;
    Eigen::Matrix4d T_front = createTransformationMatrix(rpy_front,xyz_front);
    Eigen::Matrix4d T_left = createTransformationMatrix(rpy_left,xyz_left);
    Eigen::Matrix4d T_right = createTransformationMatrix(rpy_right,xyz_right);

    pcl::PointCloud<pcl::PointXYZI>::Ptr front_cloud{new pcl::PointCloud<pcl::PointXYZI>()};
    pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloud{new pcl::PointCloud<pcl::PointXYZI>()};
    pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud{new pcl::PointCloud<pcl::PointXYZI>()};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_front_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_left_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    //rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}