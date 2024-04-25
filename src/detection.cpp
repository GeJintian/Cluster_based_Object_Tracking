#include "detection/detection.hpp"
using namespace std;

#define VEHICLE_LENGTH 7

MultiVehicleDetection::MultiVehicleDetection()
: Node("detection_node")
{
    sub_lidar_cloud_.subscribe(this, "/nonground");
    sub_radar_cloud_.subscribe(this, "/radar_fusion_points");
    //sub_lidar_cloud_.registerCallback(std::bind(&MultiVehicleDetection::topic1_callback, this, std::placeholders::_1));
    //sub_radar_cloud_.registerCallback(std::bind(&MultiVehicleDetection::topic2_callback, this, std::placeholders::_1));

    synchronizer_.reset(new Sync(SyncPolicy(100), sub_lidar_cloud_, sub_radar_cloud_));
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration(std::chrono::duration<double>(0.3)));
    synchronizer_->registerCallback(std::bind(&MultiVehicleDetection::detection_callback, this, std::placeholders::_1, std::placeholders::_2));

    vectornav_subscriber_ = this->create_subscription<a2rl_bs_msgs::msg::VectornavIns>("/a2rl/vn/ins", 10, std::bind(&MultiVehicleDetection::vectornav_callback, this, std::placeholders::_1));

    vis_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/moving_objects", 10);
    vis_radar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/moving_radar", 10);
    RCLCPP_INFO(this->get_logger(),"Finish detection initialization...");
    //KalmanFilterTracking * tracker;
}

MultiVehicleDetection::~MultiVehicleDetection(){}

void MultiVehicleDetection::topic1_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &lidar_msg)
{
    RCLCPP_INFO(this->get_logger(),"lidar_msg Current time: %ld seconds and %ld nanoseconds", 
    lidar_msg->header.stamp.sec, 
    lidar_msg->header.stamp.nanosec % 1000000000);
}

void MultiVehicleDetection::topic2_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &lidar_msg)
{
    RCLCPP_INFO(this->get_logger(),"radar_msg Current time: %ld seconds and %ld nanoseconds", 
    lidar_msg->header.stamp.sec, 
    lidar_msg->header.stamp.nanosec % 1000000000);
}

void MultiVehicleDetection::detection_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &lidar_msg, const sensor_msgs::msg::PointCloud2::ConstPtr &radar_msg){
    is_frozen = true;
    RCLCPP_INFO(this->get_logger(),"detection callback functioning ...");
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<RadarPointType>::Ptr radar_cloud(new pcl::PointCloud<RadarPointType>);
    pcl::fromROSMsg(*lidar_msg, *lidar_cloud);
    pcl::fromROSMsg(*radar_msg, *radar_cloud);
    std::cout<<"rtk velocity x,y,z "<<latest_vn_msg_.velocity_body_ins.x<<" , "<<latest_vn_msg_.velocity_body_ins.y<<" , "<<latest_vn_msg_.velocity_body_ins.z<<std::endl;
    clustering_filter(lidar_cloud, radar_cloud, latest_vn_msg_.velocity_body_ins.x, latest_vn_msg_.velocity_body_ins.y);
    //clustering_filter(lidar_cloud, radar_cloud, 0, 0);

    is_frozen = false;
}

void MultiVehicleDetection::vectornav_callback(const a2rl_bs_msgs::msg::VectornavIns::SharedPtr msg) {
    if (!is_frozen) latest_vn_msg_ = *msg;
}

void MultiVehicleDetection::clustering_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar, pcl::PointCloud<RadarPointType>::Ptr cloud_radar, double init_x, double init_y){
    //RCLCPP_INFO(this->get_logger(),"clustering filter ...");
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<RadarPointType>::Ptr radar_filtered(new pcl::PointCloud<RadarPointType>);
    
    for (const auto& point : *cloud_lidar) {
        if (point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.x > -30 && point.x < 80 && point.y > -50 && point.y < 50) {
            lidar_filtered->push_back(point);
        }
    }

    for (const auto& point : *cloud_radar){
        if (point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.x > -50 && point.x < 50 && point.y > -50 && point.y < 50) {
            radar_filtered->push_back(point);
        }
    }
    //RCLCPP_INFO(this->get_logger(),"after creating clouds ...");
    // for radar
    pcl::PointCloud<RadarPointType>::Ptr radar_outlier(new pcl::PointCloud<RadarPointType>);
    VAP(radar_filtered, radar_outlier, init_x, init_y); // consider merging ego velocity into VAP
    //std::cout<<"before vap "<<radar_filtered->size()<<std::endl;
    //std::cout<<"after vap "<<radar_outlier->size()<<std::endl;
    //RCLCPP_INFO(this->get_logger(),"after vap ...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    radar2lidar_(radar_xyz, radar_outlier);
    //RCLCPP_INFO(this->get_logger(),"after converting radar to xyz ...");
    pcl::search::KdTree<pcl::PointXYZ>::Ptr radar_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    radar_tree->setInputCloud(radar_xyz);
    std::vector<pcl::PointIndices> radar_cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(1);
    ec.setMinClusterSize(3); //TODO: change this parameter
    ec.setSearchMethod(radar_tree);
    ec.setInputCloud(radar_xyz);
    ec.extract(radar_cluster_indices);
    //RCLCPP_INFO(this->get_logger(),"after radar tree ...");
    // for lidar
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_final(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> sor;

    sor.setInputCloud(lidar_filtered);
    sor.setLeafSize(0.1f, 0.1f, 0.05f);
    sor.filter(*lidar_final);
    //RCLCPP_INFO(this->get_logger(),"after lidar gridvoxel ...");
    pcl::search::KdTree<pcl::PointXYZI>::Ptr lidar_tree(new pcl::search::KdTree<pcl::PointXYZI>);
    lidar_tree->setInputCloud(lidar_final);

    std::vector<pcl::PointIndices> lidar_cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> er;
    er.setClusterTolerance(0.3);
    er.setMinClusterSize(50); //TODO: change this parameters
    er.setSearchMethod(lidar_tree);
    er.setInputCloud(lidar_final);
    er.extract(lidar_cluster_indices);
    //RCLCPP_INFO(this->get_logger(),"after lidar tree ...");
    //TODO: maybe we also need to calculate the norm vector of each cluster
    std::vector<LidarClusterProperties> lidar_cluster_properties;
    for (std::vector<pcl::PointIndices>::const_iterator it = lidar_cluster_indices.begin(); it != lidar_cluster_indices.end(); ++it)
    {
        double x=0;
        double y=0;
        double z=0;
        double minx = 1000;
        double miny = 1000;
        double maxx = -1000;
        double maxy = -1000;
        double count = 0;
        bool valid = true;
        for (const auto& idx : it->indices){
            x += (*lidar_final)[idx].x;
            y += (*lidar_final)[idx].y;
            z += (*lidar_final)[idx].z;
            if (minx > (*lidar_final)[idx].x){
                minx = (*lidar_final)[idx].x;
            }
            if (miny > (*lidar_final)[idx].y){
                miny = (*lidar_final)[idx].y;
            }
            if (maxx < (*lidar_final)[idx].x){
                maxx = (*lidar_final)[idx].x;
            }
            if (maxy < (*lidar_final)[idx].y){
                maxy = (*lidar_final)[idx].y;
            }
            count = count + 1;
            if (maxx - minx > VEHICLE_LENGTH || maxy - miny > VEHICLE_LENGTH){
                valid = false;
                break;
            }
        }
        if(!valid) continue;
        //RCLCPP_INFO(this->get_logger(),"in lidar cluster ...");
        lidar_cluster_properties.push_back(LidarClusterProperties(x/count, y/count, z/count, minx, maxx, miny, maxy));
        for (const auto& idx : it->indices){
            lidar_cluster_properties[lidar_cluster_properties.size()-1].merged_cloud_->push_back((*lidar_final)[idx]);
        }
    }
    //RCLCPP_INFO(this->get_logger(),"after lidar cluster ...");
    std::vector<RadarClusterProperties> radar_cluster_properties;
    for (std::vector<pcl::PointIndices>::const_iterator it = radar_cluster_indices.begin(); it != radar_cluster_indices.end(); ++it)
    {
        double x=0;
        double y=0;
        double z=0;
        double minx = 1000;
        double miny = 1000;
        double maxx = -1000;
        double maxy = -1000;
        double count = 0;
        double vd = 0;
        for (const auto& idx : it->indices){
            x += (*radar_outlier)[idx].x;
            y += (*radar_outlier)[idx].y;
            z += (*radar_outlier)[idx].z;
            vd += (*radar_outlier)[idx].velocity;
            if (minx > (*radar_outlier)[idx].x){
                minx = (*radar_outlier)[idx].x;
            }
            if (miny > (*radar_outlier)[idx].y){
                miny = (*radar_outlier)[idx].y;
            }
            if (maxx < (*radar_outlier)[idx].x){
                maxx = (*radar_outlier)[idx].x;
            }
            if (maxy < (*radar_outlier)[idx].y){
                maxy = (*radar_outlier)[idx].y;
            }
            count = count + 1;
        }
        radar_cluster_properties.push_back(RadarClusterProperties(x/count, y/count, z/count, vd/count));
        for (const auto& idx : it->indices){
            radar_cluster_properties[radar_cluster_properties.size()-1].merged_cloud_->push_back((*radar_outlier)[idx]);
        }
    }
    //RCLCPP_INFO(this->get_logger(),"after radar cluster ...");
    // KNN search
    std::vector<int> associations(radar_cluster_properties.size(), -1);
    CLUSTER_ASSOCIATION_KNN(lidar_cluster_properties, radar_cluster_properties, associations);

    //std::cout<<"Radar clustering returns "<<radar_cluster_properties.size()<<" clusters"<<std::endl;
    //std::cout<<"Lidar clustering returns "<<lidar_cluster_properties.size()<<" clusters"<<std::endl;
    //RCLCPP_INFO(this->get_logger(),"after knn ...");
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto &idx : associations){
        if(idx > -1) *merged_cloud_ += *(lidar_cluster_properties[idx].merged_cloud_);
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*merged_cloud_, output);
    output.header.frame_id = "map";
    vis_publisher_->publish(output);
    //RCLCPP_INFO(this->get_logger(),"Pub lidar ...");
    pcl::PointCloud<RadarPointType>::Ptr merged_radar_(new pcl::PointCloud<RadarPointType>);
    for(auto &idx : radar_cluster_properties){
        *merged_radar_ += *(idx.merged_cloud_);
    }
    // for (std::vector<pcl::PointIndices>::const_iterator it = radar_cluster_indices.begin(); it != radar_cluster_indices.end(); ++it)
    // {
    //     for (const auto& idx : it->indices)
    //         merged_radar_->push_back((*radar_outlier)[idx]);
    //     merged_radar_->width = merged_radar_->size();
    //     merged_radar_->height = 1;
    //     merged_radar_->is_dense = true;
    // }

    sensor_msgs::msg::PointCloud2 radaroutput;
    pcl::toROSMsg(*radar_outlier, radaroutput);
    radaroutput.header.frame_id = "map";
    vis_radar_publisher_->publish(radaroutput);
    //RCLCPP_INFO(this->get_logger(),"Pub radar ...");
}