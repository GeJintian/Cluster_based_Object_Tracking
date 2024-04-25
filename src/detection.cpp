#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include "detection/detection.hpp"
#include <signal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <sstream>
// #include <pcl/common/parallel_for.h>

//using PointType = pcl::PointXYZI;
using namespace std;


template<typename PointT>
MOT<PointT>::MOT(const rclcpp::NodeOptions &options)
    : Node("MOT", options) {
    // Init ROS related

    this->declare_parameter<double>("sensor_height", sensor_height_);
    this->declare_parameter<int>("num_iter", num_iter_);
    this->declare_parameter<int>("num_lpr", num_lpr_);
    this->declare_parameter<int>("num_min_pts", num_min_pts_);
    this->declare_parameter<double>("th_seeds", th_seeds_);
    this->declare_parameter<double>("th_dist", th_dist_);
    this->declare_parameter<double>("max_range", max_range_);
    this->declare_parameter<double>("min_range", min_range_);
    this->declare_parameter<double>("uprightness_thr", uprightness_thr_);
    this->declare_parameter<double>("adaptive_seed_selection_margin", adaptive_seed_selection_margin_);
    this->declare_parameter<double>("RNR_ver_angle_thr", RNR_ver_angle_thr_);

    this->declare_parameter<int>("num_zones", num_zones_);
    this->declare_parameter<double>("th_seeds_v", th_seeds_v_);
    this->declare_parameter<double>("th_dist_v", th_dist_v_);
    this->declare_parameter<double>("RNR_intensity_thr", RNR_intensity_thr_);
    this->declare_parameter<int>("core_num", core_num);
    this->declare_parameter<std::string>("cloud_topic", cloud_topic);
    this->declare_parameter<std::string>("left_bound_file", left_bound_file);
    this->declare_parameter<std::string>("right_bound_file", right_bound_file);
    this->declare_parameter<std::string>("frame_id", frame_id_);
    this->declare_parameter<bool>("verbose", verbose_);
    this->declare_parameter<bool>("display_time", display_time_);
    this->declare_parameter<bool>("display_position", display_position_);
    this->declare_parameter<bool>("visualize", visualize_);
    this->declare_parameter<bool>("non_ground_pub", non_ground_pub_);
    this->declare_parameter<bool>("objects_cloud_pub", objects_cloud_pub_);
    this->declare_parameter<double>("xy_leaf_size", xy_leaf_size_);
    this->declare_parameter<double>("z_leaf_size", z_leaf_size_);
    this->declare_parameter<double>("cluster_min_dist", cluster_min_dist_);
    this->declare_parameter<int>("cluster_min_size", cluster_min_size_);

    this->get_parameter<double>("sensor_height", sensor_height_);
    this->get_parameter<int>("num_iter", num_iter_);
    this->get_parameter<int>("num_lpr", num_lpr_);
    this->get_parameter<int>("num_min_pts", num_min_pts_);
    this->get_parameter<double>("th_seeds", th_seeds_);
    this->get_parameter<double>("th_dist", th_dist_);
    this->get_parameter<double>("max_range", max_range_);
    this->get_parameter<double>("min_range", min_range_);
    this->get_parameter<double>("uprightness_thr", uprightness_thr_);
    this->get_parameter<double>("adaptive_seed_selection_margin", adaptive_seed_selection_margin_);
    this->get_parameter<double>("RNR_ver_angle_thr", RNR_ver_angle_thr_);

    this->get_parameter<int>("num_zones", num_zones_);
    this->get_parameter<double>("th_seeds_v", th_seeds_v_);
    this->get_parameter<double>("th_dist_v", th_dist_v_);
    this->get_parameter<double>("RNR_intensity_thr", RNR_intensity_thr_);
    this->get_parameter<int>("core_num", core_num);
    this->get_parameter<std::string>("cloud_topic", cloud_topic);
    this->get_parameter<std::string>("left_bound_file", left_bound_file);
    this->get_parameter<std::string>("right_bound_file", right_bound_file);
    this->get_parameter<std::string>("frame_id", frame_id_);
    this->get_parameter<bool>("verbose", verbose_);
    this->get_parameter<bool>("display_time", display_time_);
    this->get_parameter<bool>("display_position", display_position_);
    this->get_parameter<bool>("visualize", visualize_);
    this->get_parameter<bool>("non_ground_pub", non_ground_pub_);
    this->get_parameter<bool>("objects_cloud_pub", objects_cloud_pub_);
    this->get_parameter<double>("xy_leaf_size", xy_leaf_size_);
    this->get_parameter<double>("z_leaf_size", z_leaf_size_);
    this->get_parameter<double>("cluster_min_dist", cluster_min_dist_);
    this->get_parameter<int>("cluster_min_size", cluster_min_size_);

    RCLCPP_INFO_STREAM(this->get_logger(), "Inititalizing MOT...");
    RCLCPP_INFO_STREAM(this->get_logger(), "Sensor Height: " << sensor_height_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Num of Iteration: " <<  num_iter_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Num of LPR: " <<  num_lpr_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Num of min. points: " << num_min_pts_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Seeds Threshold: "<< th_seeds_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Distance Threshold: "<< th_dist_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Max. range:: " << max_range_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Min. range:: " << min_range_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Normal vector threshold: " <<  uprightness_thr_);
    RCLCPP_INFO_STREAM(this->get_logger(), "adaptive_seed_selection_margin: " << adaptive_seed_selection_margin_);
    RCLCPP_INFO_STREAM(this->get_logger(), "RNR_ver_angle_thr: " << RNR_ver_angle_thr_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Num. zones: " << num_zones_);
    RCLCPP_INFO_STREAM(this->get_logger(), "cloud_topic: " << cloud_topic);
    RCLCPP_INFO_STREAM(this->get_logger(), "frame_id: " << frame_id_);

    // CZM denotes 'Concentric Zone Model'. Please refer to our paper
    num_sectors_each_zone_ = std::vector<long>{8, 8, 8, 8};
    new_num_sectors_each_zone_   = std::vector<long>{8, 8, 8, 8}; 
    num_rings_each_zone_   = std::vector<long>{8, 8, 8, 8};
    new_num_rings_each_zone_   = std::vector<long>{8, 8, 8, 8};
    elevation_thr_ = std::vector<double>{0.0, 0.0, 0.0, 0.0};
    new_elevation_thr_ = std::vector<double>{0.0, 0.0, 0.0, 0.0};
    flatness_thr_  = std::vector<double>{0.0, 0.0, 0.0, 0.0};
    new_flatness_thr_  = std::vector<double>{0.0, 0.0, 0.0, 0.0};

    RCLCPP_INFO(rclcpp::get_logger("Detector"), "Num. zones: %d", num_zones_);

    if (num_zones_ != 4 || num_sectors_each_zone_.size() != num_rings_each_zone_.size()) {
        throw invalid_argument("Some parameters are wrong! Check the num_zones and num_rings/sectors_each_zone");
    }
    if (elevation_thr_.size() != flatness_thr_.size()) {
        throw invalid_argument("Some parameters are wrong! Check the elevation/flatness_thresholds");
    }

    cout << (boost::format("Num. sectors: %d, %d, %d, %d") % num_sectors_each_zone_[0] % num_sectors_each_zone_[1] %
                num_sectors_each_zone_[2] %
                num_sectors_each_zone_[3]).str() << endl;
    cout << (boost::format("Num. rings: %01d, %01d, %01d, %01d") % num_rings_each_zone_[0] %
                num_rings_each_zone_[1] %
                num_rings_each_zone_[2] %
                num_rings_each_zone_[3]).str() << endl;
    cout << (boost::format("elevation_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % elevation_thr_[0] % elevation_thr_[1] %
                elevation_thr_[2] %
                elevation_thr_[3]).str() << endl;
    cout << (boost::format("flatness_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % flatness_thr_[0] % flatness_thr_[1] %
                flatness_thr_[2] %
                flatness_thr_[3]).str() << endl;
    num_rings_of_interest_ = elevation_thr_.size();

    int num_polygons = std::inner_product(num_rings_each_zone_.begin(), num_rings_each_zone_.end(), num_sectors_each_zone_.begin(), 0);
    revert_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

    pub_revert_pc = Node::create_publisher<sensor_msgs::msg::PointCloud2>("plane", 100);
    pub_reject_pc = Node::create_publisher<sensor_msgs::msg::PointCloud2>("plane", 100);
    pub_normal = Node::create_publisher<sensor_msgs::msg::PointCloud2>("plane", 100);
    pub_noise = Node::create_publisher<sensor_msgs::msg::PointCloud2>("plane", 100);
    pub_vertical = Node::create_publisher<sensor_msgs::msg::PointCloud2>("plane", 100);

    min_range_z2_ = (7 * min_range_ + max_range_) / 8.0;
    min_range_z3_ = (3 * min_range_ + max_range_) / 4.0;
    min_range_z4_ = (min_range_ + max_range_) / 2.0;
    min_ranges_ = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_};
    ring_sizes_ = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                    (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                    (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                    (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
    sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0), 2 * M_PI / num_sectors_each_zone_.at(1),
                    2 * M_PI / num_sectors_each_zone_.at(2),
                    2 * M_PI / num_sectors_each_zone_.at(3)};

    cout << "INITIALIZATION COMPLETE" << endl;
    for (int i = 0; i < num_zones_; i++) {
        Zone z;
        initialize_zone(z, num_sectors_each_zone_[i], num_rings_each_zone_[i]);
        ConcentricZoneModel_.push_back(z);
    }

    pub_curbside = Node::create_publisher<sensor_msgs::msg::PointCloud2>("curbside", 100);
    pub_object = Node::create_publisher<sensor_msgs::msg::PointCloud2>("objects", 100);

    pub_cluster_ = Node::create_publisher<a2rl_bs_msgs::msg::Perception>("/a2rl/perception/state", 100);
    pub_cloud = Node::create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 100);
    pub_ground = Node::create_publisher<sensor_msgs::msg::PointCloud2>("ground", 100);
    pub_non_ground = Node::create_publisher<sensor_msgs::msg::PointCloud2>("nonground", 100);
    sub_cloud = Node::create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&MOT<PointT>::callbackCloud, this, std::placeholders::_1));
    vectornav_subscriber_ = this->create_subscription<a2rl_bs_msgs::msg::VectornavIns>("/a2rl/vn/ins", 10, std::bind(&MOT::vectornav_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MOT<PointT>::parametersCallback, this, std::placeholders::_1));

    //Initialize bound frenet
    std::vector<detection_helper::CartesianPoint> left_reference;
    std::vector<detection_helper::CartesianPoint> right_reference;
    std::cout<<core_num<<std::endl;
    omp_set_num_threads(core_num);
    ifstream left_file(left_bound_file);
    if (!left_file) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open left boundary file\n");
    }
    string line;
    while (getline(left_file, line)) { 
        istringstream ss(line);
        string field;
        double x;
        double y;
        int index = 0;

        while (getline(ss, field, ',')) {
            double value = stod(field);
            if (index == 0) {
                x = value;
            } else if (index == 1) {
                y = value;
            }
            index++;
        }
        left_reference.push_back(detection_helper::CartesianPoint{x, y});
    }
    left_file.close(); 

    ifstream right_file(right_bound_file);
    if (!right_file) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open right boundary file\n");
    }
    while (getline(right_file, line)) { 
        istringstream ss(line);
        string field;
        double x;
        double y;
        int index = 0;

        while (getline(ss, field, ',')) {
            double value = stod(field);
            if (index == 0) {
                x = value;
            } else if (index == 1) {
                y = value;
            }
            index++;
        }
        right_reference.push_back(detection_helper::CartesianPoint{x, y});
    }
    right_file.close(); 

    // detection_helper::Reference l(left_reference);
    // detection_helper::Reference r(right_reference);
    left_bound_ptr = std::make_shared<detection_helper::Reference>(left_reference);
    right_bound_ptr = std::make_shared<detection_helper::Reference>(right_reference);
};

template<typename PointT>
void MOT<PointT>::vectornav_callback(const a2rl_bs_msgs::msg::VectornavIns::SharedPtr msg) {
    if (!is_frozen) latest_vn_msg_ = *msg;
}

template<typename PointT>
Eigen::Vector2d MOT<PointT>::local2enu(double x, double y){
    Eigen::Vector4d eigen_point(x, y, 0, 1);
    Eigen::Vector4d result = transformMatrix*eigen_point;
    Eigen::Vector2d return_value(result(0),result(1));
    return return_value;
}

template<typename PointT>
void MOT<PointT>::initialize_zone(Zone &z, int num_sectors, int num_rings) {
    z.clear();
    pcl::PointCloud<PointT> cloud;
    cloud.reserve(1000);
    Ring ring;
    for (int i = 0; i < num_sectors; i++) {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++) {
        z.emplace_back(ring);
    }
}

template<typename PointT>
void MOT<PointT>::flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings) {
    for (int i = 0; i < num_sectors; i++) {
        for (int j = 0; j < num_rings; j++) {
            if (!patches[j][i].points.empty()) patches[j][i].points.clear();
        }
    }
}

template<typename PointT>
void MOT<PointT>::flush_patches(vector<Zone> &czm) {
    for (int k = 0; k < num_zones_; k++) {
        for (int i = 0; i < num_rings_each_zone_[k]; i++) {
            for (int j = 0; j < num_sectors_each_zone_[k]; j++) {
                if (!czm[k][i][j].points.empty()) czm[k][i][j].points.clear();
            }
        }
    }

    if( verbose_ ) cout << "Flushed patches" << endl;
}

template<typename PointT>
void MOT<PointT>::estimate_plane(const pcl::PointCloud<PointT> &ground) {
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));

    if (normal_(2) < 0) { for(int i=0; i<3; i++) normal_(i) *= -1; }

    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}

template<typename PointT>
void MOT<PointT>::extract_initial_seeds(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds, double th_seed) {
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < adaptive_seed_selection_margin_ * sensor_height_) {
                ++init_idx;
            } else {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seed) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

template<typename PointT>
void MOT<PointT>::extract_initial_seeds(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds) {
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    int init_idx = 0;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < adaptive_seed_selection_margin_ * sensor_height_) {
                ++init_idx;
            } else {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

template<typename PointT>
void MOT<PointT>::reflected_noise_removal(pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_nonground)
{
    for (int i=0; i<cloud_in.size(); i++)
    {
        double r = sqrt( cloud_in[i].x*cloud_in[i].x + cloud_in[i].y*cloud_in[i].y );
        double z = cloud_in[i].z;
        double ver_angle_in_deg = atan2(z, r)*180/M_PI;

        if ( ver_angle_in_deg < RNR_ver_angle_thr_ && z < -sensor_height_-0.8 && cloud_in[i].intensity < RNR_intensity_thr_)
        {
            cloud_nonground.push_back(cloud_in[i]);
            noise_pc_.push_back(cloud_in[i]);
            noise_idxs_.push(i);
        }
    }

    if (verbose_) cout << "[ RNR ] Num of noises : " << noise_pc_.points.size() << endl;
}

template<typename PointT>
rcl_interfaces::msg::SetParametersResult MOT<PointT>::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Param updated: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
        if(param.get_name() == "cloud_topic" or param.get_name() == "frame_id")
        {
            cloud_topic = param.value_to_string();
            sub_cloud = Node::create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&MOT<PointT>::callbackCloud, this, std::placeholders::_1));    
        }
        if(param.get_name()=="core_num"){
            core_num = param.as_int();
        }
        if(param.get_name() == "left_bound_file"){
            left_bound_file = param.value_to_string();
        }
        if(param.get_name() == "right_bound_file"){
            right_bound_file = param.value_to_string();
        }
        if(param.get_name() == "sensor_height")
        {
            sensor_height_ = param.as_double();
        }
        if(param.get_name() == "num_iter")
        {
            num_iter_ = param.as_int();
        }
        if(param.get_name() == "num_lpr")
        {
            num_lpr_ = param.as_int();
        }
        if(param.get_name() == "num_min_pts")
        {
            num_min_pts_ = param.as_int();
        }
        if(param.get_name() == "th_seeds")
        {
            th_seeds_ = param.as_double();
        }
        if(param.get_name() == "th_dist")
        {
            th_dist_ = param.as_double();
        }
        if(param.get_name() == "max_range")
        {
            max_range_ = param.as_double();
        }
        if(param.get_name() == "min_range")
        {
            min_range_ = param.as_double();
        }
        if(param.get_name() == "uprightness_thr")
        {
            uprightness_thr_ = param.as_double();
        }
        if(param.get_name() == "adaptive_seed_selection_margin")
        {
            adaptive_seed_selection_margin_ = param.as_double();
        }
        if(param.get_name() == "RNR_ver_angle_thr")
        {
            RNR_ver_angle_thr_ = param.as_double();
        }
        if(param.get_name() == "RNR_intensity_thr")
        {
            RNR_intensity_thr_ = param.as_double();
        }
        if(param.get_name() == "num_zones")
        {
            num_zones_ = param.as_int();
        }
        if(param.get_name() == "th_seeds_v")
        {
            th_seeds_v_ = param.as_double();
        }
        if(param.get_name() == "th_dist_v")
        {
            th_dist_v_ = param.as_double();
        }
        if(param.get_name() == "verbose")
        {
            verbose_ = param.as_bool();
        }
        if(param.get_name() == "display_time")
        {
            display_time_ = param.as_bool();
        }
        if(param.get_name() == "display_position")
        {
            display_position_ = param.as_bool();
        }
        if(param.get_name() == "visualize")
        {
            visualize_ = param.as_bool();
        }
        if(param.get_name() == "non_ground_pub")
        {
            non_ground_pub_ = param.as_bool();
        }
        if(param.get_name() == "objects_cloud_pub")
        {
            objects_cloud_pub_ = param.as_bool();
        }
        if(param.get_name() == "xy_leaf_size")
        {
            xy_leaf_size_ = param.as_double();
        }
        if(param.get_name() == "z_leaf_size")
        {
            z_leaf_size_ = param.as_double();
        }
        if(param.get_name() == "cluster_min_dist")
        {
            cluster_min_dist_ = param.as_double();
        }
        if(param.get_name() == "cluster_min_size")
        {
            cluster_min_size_ = param.as_int();
        }
    }

    return result;
}

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/

// Functions for extract curbside
template<typename PointT>
void MOT<PointT>::extract_curbside(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_curb){
    cloud_curb.clear();
    // pcl::PointCloud<PointT> cloud_filtered;

    pcl::PointCloud<PointT> cloud_filtered;
    pcl::PointCloud<PointT> cloud_temp;
    pcl::PointCloud<PointT> cloud_final;



    for (const auto& point : cloud_in) {
    if (point.z >= Z_LOWER_BOUND && point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.x > 0 && point.x < 100 && point.y > -50 && point.y < 50) {
        cloud_temp.push_back(point);
    }
    }

    // pcl::VoxelGrid<PointT> sor;
    // sor.setInputCloud(cloud_temp.makeShared());
    // sor.setLeafSize(0.1f, 0.1f, 0.05f);
    // sor.filter(cloud_final);
    cloud_final = cloud_temp;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_final.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(50);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_final.makeShared());
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for (const auto& idx : it->indices)
            cloud_filtered.push_back((cloud_final)[idx]);

        cloud_filtered.width = cloud_filtered.size();
        cloud_filtered.height = 1;
        cloud_filtered.is_dense = true;
    }

    int bin_num = std::floor(SEARCH_RANGE / ANGLE_RES);
    
    std::vector<std::vector<PointT>> bins(bin_num);
    
    for (const auto& point : cloud_filtered) {
        //if (point.z >= Z_LOWER_BOUND && point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND) {
            //double angleRadians = std::atan2(point.y, point.x); // x-axis search
            double angleRadians = std::atan2(point.y, point.x);
            double angleDegrees = angleRadians * (180.0 / M_PI);
            angleDegrees = fmod(angleDegrees + 360.0, 360.0);
            //if (angleDegrees > 180.0) angleDegrees -= 360.0;
            if (angleDegrees < SEARCH_RANGE) {
                int idx = std::floor((angleDegrees) / ANGLE_RES);
                if (idx >= 0 && idx < bin_num) {
                    bins[idx].push_back(point);
                }
            }
        //}
    }
    
    for (int idx = 0; idx < bin_num; ++idx) {
        if (!bins[idx].empty()) {
            auto min_it = std::min_element(bins[idx].begin(), bins[idx].end(),
                [](const PointT& a, const PointT& b) {
                    return (a.x * a.x + a.y * a.y) < (b.x * b.x + b.y * b.y);
                });
            cloud_curb.push_back(*min_it);
        }
    }
}

template<typename PointT>
void MOT<PointT>::publish_single_perception(std::vector<LidarClusterProperties> &collections, builtin_interfaces::msg::Time time){
    RCLCPP_INFO(this->get_logger(),"ready to publish");
    if(collections.empty()) return;
    double closest_value = 100000;
    double closest_x;
    double closest_y;
    double center_x;
    double center_y;

    for(const auto &property: collections){
        double x;
        double y;
        double value=100000;
        for(const auto &p:*(property.merged_cloud_)){
            double dist = sqrt(p.x*p.x+p.y*p.y);
            if(value>dist){
                value = dist;
                x = p.x;
                y = p.y;
            }
        }
        if(closest_value>value){
            closest_x = x;
            closest_y = y;
            center_x = property.center_x;
            center_y = property.center_y;
        }
    }
    a2rl_bs_msgs::msg::Perception output;
    a2rl_bs_msgs::msg::Timestamp timestamp;
    a2rl_bs_msgs::msg::CartesianFrame center;
    a2rl_bs_msgs::msg::CartesianFrame speed;
    a2rl_bs_msgs::msg::CartesianFrame close;
    center.x = center_x;
    center.y = center_y;
    close.x = closest_x;
    close.y = closest_y;
    speed.x = 0;
    speed.y = 0;
    timestamp.nanoseconds = time.nanosec;
    output.timestamp = timestamp;
    output.speed = speed;
    output.position = center;
    output.closed_point = close;
    pub_cluster_->publish(output);
    RCLCPP_INFO(this->get_logger(),"finish publish");
}

template<typename PointT>
void MOT<PointT>::object_detection(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_object, builtin_interfaces::msg::Time time){
    cloud_object.clear();
    // pcl::PointCloud<PointT> cloud_filtered;

    pcl::PointCloud<PointT> cloud_filtered;
    pcl::PointCloud<PointT> cloud_temp;
    pcl::PointCloud<PointT> cloud_final;


    for (const auto& point : cloud_in) {
    if (point.z >= Z_LOWER_BOUND && point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.x > -50 && point.x < 50 && point.y > -30 && point.y < 30) {
        cloud_temp.push_back(point);
    }
    }

    // double grid_start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
    // pcl::VoxelGrid<PointT> sor;
    // sor.setInputCloud(cloud_temp.makeShared());
    // sor.setLeafSize(0.2f, 0.2f, 0.05f);
    // sor.filter(cloud_final);
    cloud_final = cloud_temp;

    // if(display_time_){
    //     RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;32m" << "Grid Voxel take time: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds() - grid_start << " sec)" << "\033[0m");
    // }

    double tree_start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    //typename pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>);
    tree->setInputCloud(cloud_final.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_min_dist_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_final.makeShared());
    ec.extract(cluster_indices);

    if(display_time_){
        RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;32m" << "Tree search take time: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds() - tree_start << " sec)" << "\033[0m");
    }

    double cluster_start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for (const auto& idx : it->indices)
            cloud_filtered.push_back((cloud_final)[idx]);

        cloud_filtered.width = cloud_filtered.size();
        cloud_filtered.height = 1;
        cloud_filtered.is_dense = true;
    }

    std::vector<LidarClusterProperties> lidar_cluster_properties;
    //#pragma omp parallel for default(shared)
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
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
            x += (cloud_final)[idx].x;
            y += (cloud_final)[idx].y;
            z += (cloud_final)[idx].z;
            if (minx > (cloud_final)[idx].x){
                minx = (cloud_final)[idx].x;
            }
            if (miny > (cloud_final)[idx].y){
                miny = (cloud_final)[idx].y;
            }
            if (maxx < (cloud_final)[idx].x){
                maxx = (cloud_final)[idx].x;
            }
            if (maxy < (cloud_final)[idx].y){
                maxy = (cloud_final)[idx].y;
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
            lidar_cluster_properties[lidar_cluster_properties.size()-1].merged_cloud_->push_back((cloud_final)[idx]);
        }
    }

    if(display_time_){
        RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;32m" << "Cluster search take time: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds() - cluster_start << " sec)" << "\033[0m");
    }
    
    detection_helper::FrenetPoint l_sd;
    detection_helper::FrenetPoint r_sd;

    std::vector<LidarClusterProperties> cluster_collection;

    double frenet_start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
    if(display_position_){
    RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;33m" << "Current ego position is at (" <<transformMatrix(0,3)<<", "<< transformMatrix(1,3)<< ")" << "\033[0m");}
    //#pragma omp parallel for
    for(int idx = 0; idx < lidar_cluster_properties.size(); idx++){
        Eigen::Vector2d p_enu = local2enu(lidar_cluster_properties[idx].center_x, lidar_cluster_properties[idx].center_y);
        l_sd = left_bound_ptr->sd({p_enu(0), p_enu(1)}, left_s_guess-150);
        r_sd = right_bound_ptr->sd({p_enu(0), p_enu(1)}, right_s_guess-150);
        if(l_sd.d <0 && r_sd.d>0){
            if(display_position_){
            RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;32m" <<"Object is at position ("<<lidar_cluster_properties[idx].center_x<<", "<<lidar_cluster_properties[idx].center_y<<"), enu is at ("<<p_enu(0)<<", "<<p_enu(1)<< "), left l_sd is (" <<l_sd.s<<", "<< l_sd.d << "), right r_sd is (" <<r_sd.s<<", "<< r_sd.d << ")" << "\033[0m");}
            //#pragma omp critical
            {
                cloud_object += *(lidar_cluster_properties[idx].merged_cloud_);
                cluster_collection.push_back(lidar_cluster_properties[idx]);
            }
        }
    }
    if(display_time_){
        RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;32m" << "Frenet search take time: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds() - frenet_start << " sec)" << "\033[0m");
    }
    left_s_guess = l_sd.s;
    right_s_guess = r_sd.s;
    publish_single_perception(cluster_collection, time);
    //cloud_object = cloud_filtered;
}


template<typename PointT>
void MOT<PointT>::estimate_ground(
        pcl::PointCloud<PointT> cloud_in,
        pcl::PointCloud<PointT> &cloud_ground,
        pcl::PointCloud<PointT> &cloud_nonground,
        double &time_taken) {

    unique_lock<recursive_mutex> lock(mutex_);
    static double start, t0, t1, t2, end;

    double pca_time_ = 0.0;
    double t_revert = 0.0;
    double t_total_ground = 0.0;
    double t_total_estimate = 0.0;

    // start = ros::Time::now().toSec();
    start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

    cloud_ground.clear();
    cloud_nonground.clear();

    // 1. Reflected Noise Removal (RNR)
    if (enable_RNR_) reflected_noise_removal(cloud_in, cloud_nonground);

    t1 = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

    // 2. Concentric Zone Model (CZM)
    flush_patches(ConcentricZoneModel_);
    pc2czm(cloud_in, ConcentricZoneModel_, cloud_nonground);

    t2 = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

    int concentric_idx = 0;

    double t_sort = 0;

    std::vector<RevertCandidate<PointT>> candidates;
    std::vector<double> ringwise_flatness;

    for (int zone_idx = 0; zone_idx < num_zones_; ++zone_idx) {

        auto zone = ConcentricZoneModel_[zone_idx];

        for (int ring_idx = 0; ring_idx < num_rings_each_zone_[zone_idx]; ++ring_idx) {
            for (int sector_idx = 0; sector_idx < num_sectors_each_zone_[zone_idx]; ++sector_idx) {

                if (zone[ring_idx][sector_idx].points.size() < num_min_pts_)
                {
                    cloud_nonground += zone[ring_idx][sector_idx];
                    continue;
                }

                // --------- region-wise sorting (faster than global sorting method) ---------------- //
                double t_sort_0 = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

                sort(zone[ring_idx][sector_idx].points.begin(), zone[ring_idx][sector_idx].points.end(), point_z_cmp<PointT>);

                double t_sort_1 = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
                t_sort += (t_sort_1 - t_sort_0);
                // ---------------------------------------------------------------------------------- //

                double t_tmp0 = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
                extract_piecewiseground(zone_idx, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);

                double t_tmp1 = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
                t_total_ground += t_tmp1 - t_tmp0;
                pca_time_ += t_tmp1 - t_tmp0;

                // Status of each patch
                // used in checking uprightness, elevation, and flatness, respectively
                const double ground_uprightness = normal_(2);
                const double ground_elevation   = pc_mean_(2, 0);
                const double ground_flatness    = singular_values_.minCoeff();
                const double line_variable      = singular_values_(1) != 0 ? singular_values_(0)/singular_values_(1) : std::numeric_limits<double>::max();

                double heading = 0.0;
                for(int i=0; i<3; i++) heading += pc_mean_(i,0)*normal_(i);


                double t_tmp2 = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

                /*
                    About 'is_heading_outside' condidition, heading should be smaller than 0 theoretically.
                    ( Imagine the geometric relationship between the surface normal vector on the ground plane and
                        the vector connecting the sensor origin and the mean point of the ground plane )

                    However, when the patch is far awaw from the sensor origin,
                    heading could be larger than 0 even if it's ground due to lack of amount of ground plane points.

                    Therefore, we only check this value when concentric_idx < num_rings_of_interest ( near condition )
                */
                bool is_upright         = ground_uprightness > uprightness_thr_;
                bool is_not_elevated    = ground_elevation < elevation_thr_[concentric_idx];
                bool is_flat            = ground_flatness < flatness_thr_[concentric_idx];
                bool is_near_zone       = concentric_idx < num_rings_of_interest_;
                bool is_heading_outside = heading < 0.0;

                /*
                    Store the elevation & flatness variables
                    for A-GLE (Adaptive Ground Likelihood Estimation)
                    and TGR (Temporal Ground Revert). More information in the paper Patchwork++.
                */
                if (is_upright && is_not_elevated && is_near_zone)
                {
                    update_elevation_[concentric_idx].push_back(ground_elevation);
                    update_flatness_[concentric_idx].push_back(ground_flatness);
                    ringwise_flatness.push_back(ground_flatness);
                }

                // Ground estimation based on conditions
                if (!is_upright)
                {
                    cloud_nonground += regionwise_ground_;
                }
                else if (!is_near_zone)
                {
                    cloud_ground += regionwise_ground_;
                }
                else if (!is_heading_outside)
                {
                    cloud_nonground += regionwise_ground_;
                }
                else if (is_not_elevated || is_flat)
                {
                    cloud_ground += regionwise_ground_;
                }
                else
                {
                    RevertCandidate<PointT> candidate(concentric_idx, sector_idx, ground_flatness, line_variable, pc_mean_, regionwise_ground_);
                    candidates.push_back(candidate);
                }
                // Every regionwise_nonground is considered nonground.
                cloud_nonground += regionwise_nonground_;

                double t_tmp3 = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
                t_total_estimate += t_tmp3 - t_tmp2;
            }

            double t_bef_revert = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

            if (!candidates.empty())
            {
                if (enable_TGR_)
                {
                    temporal_ground_revert(cloud_ground, cloud_nonground, ringwise_flatness, candidates, concentric_idx);
                }
                else
                {
                    for (size_t i=0; i<candidates.size(); i++)
                    {
                        cloud_nonground += candidates[i].regionwise_ground;
                    }
                }

                candidates.clear();
                ringwise_flatness.clear();
            }

            double t_aft_revert = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

            t_revert += t_aft_revert - t_bef_revert;

            concentric_idx++;
            //std::cout<<concentric_idx<<std::endl;
        }
    }

    double t_update = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();

    update_elevation_thr();
    update_flatness_thr();

    end = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
    time_taken = end - start;

    if (visualize_)
    {
        sensor_msgs::msg::PointCloud2 cloud_ROS;
        pcl::toROSMsg(revert_pc_, cloud_ROS);
        cloud_ROS.header.stamp = rclcpp::Clock{RCL_STEADY_TIME}.now();
        cloud_ROS.header.frame_id = cloud_in.header.frame_id;
        pub_revert_pc->publish(cloud_ROS);

        pcl::toROSMsg(reject_pc_, cloud_ROS);
        cloud_ROS.header.stamp = rclcpp::Clock{RCL_STEADY_TIME}.now();
        cloud_ROS.header.frame_id = cloud_in.header.frame_id;
        pub_reject_pc->publish(cloud_ROS);

        pcl::toROSMsg(normals_, cloud_ROS);
        cloud_ROS.header.stamp = rclcpp::Clock{RCL_STEADY_TIME}.now();
        cloud_ROS.header.frame_id = cloud_in.header.frame_id;
        pub_normal->publish(cloud_ROS);

        pcl::toROSMsg(noise_pc_, cloud_ROS);
        cloud_ROS.header.stamp = rclcpp::Clock{RCL_STEADY_TIME}.now();
        cloud_ROS.header.frame_id = cloud_in.header.frame_id;
        pub_noise->publish(cloud_ROS);

        pcl::toROSMsg(vertical_pc_, cloud_ROS);
        cloud_ROS.header.stamp = rclcpp::Clock{RCL_STEADY_TIME}.now();
        cloud_ROS.header.frame_id = cloud_in.header.frame_id;
        pub_vertical->publish(cloud_ROS);
    }

    revert_pc_.clear();
    reject_pc_.clear();
    normals_.clear();
    noise_pc_.clear();
    vertical_pc_.clear();
}

template<typename PointT>
void MOT<PointT>::update_elevation_thr(void)
{
    for (int i=0; i<num_rings_of_interest_; i++)
    {
        if (update_elevation_[i].empty()) continue;

        double update_mean = 0.0, update_stdev = 0.0;
        calc_mean_stdev(update_elevation_[i], update_mean, update_stdev);
        if (i==0) {
            elevation_thr_[i] = update_mean + 3*update_stdev;
            sensor_height_ = -update_mean;
        }
        else elevation_thr_[i] = update_mean + 2*update_stdev;

        int exceed_num = update_elevation_[i].size() - max_elevation_storage_;
        if (exceed_num > 0) update_elevation_[i].erase(update_elevation_[i].begin(), update_elevation_[i].begin() + exceed_num);
    }

    if (verbose_)
    {
        cout << "sensor height: " << sensor_height_ << endl;
        cout << (boost::format("elevation_thr_  :   %0.4f,  %0.4f,  %0.4f,  %0.4f")
                % elevation_thr_[0] % elevation_thr_[1] % elevation_thr_[2] % elevation_thr_[3]).str() << endl;
    }

    return;
}

template<typename PointT>
void MOT<PointT>::update_flatness_thr(void)
{
    for (int i=0; i<num_rings_of_interest_; i++)
    {
        if (update_flatness_[i].empty()) break;
        if (update_flatness_[i].size() <= 1) break;

        double update_mean = 0.0, update_stdev = 0.0;
        calc_mean_stdev(update_flatness_[i], update_mean, update_stdev);
        flatness_thr_[i] = update_mean+update_stdev;

        int exceed_num = update_flatness_[i].size() - max_flatness_storage_;
        if (exceed_num > 0) update_flatness_[i].erase(update_flatness_[i].begin(), update_flatness_[i].begin() + exceed_num);
    }

    if (verbose_)
    {
        cout << (boost::format("flatness_thr_   :   %0.4f,  %0.4f,  %0.4f,  %0.4f")
                % flatness_thr_[0] % flatness_thr_[1] % flatness_thr_[2] % flatness_thr_[3]).str() << endl;
    }

    return;
}

template<typename PointT>
void MOT<PointT>::temporal_ground_revert(pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground,
                                            std::vector<double> ring_flatness, std::vector<RevertCandidate<PointT>> candidates,
                                            int concentric_idx)
{
    if (verbose_) std::cout << "\033[1;34m" << "=========== Temporal Ground Revert (TGR) ===========" << "\033[0m" << endl;

    double mean_flatness = 0.0, stdev_flatness = 0.0;
    calc_mean_stdev(ring_flatness, mean_flatness, stdev_flatness);

    if (verbose_)
    {
        cout << "[" << candidates[0].concentric_idx << ", " << candidates[0].sector_idx << "]"
            << " mean_flatness: " << mean_flatness << ", stdev_flatness: " << stdev_flatness << std::endl;
    }

    for( size_t i=0; i<candidates.size(); i++ )
    {
        RevertCandidate<PointT> candidate = candidates[i];

        // Debug
        if(verbose_)
        {
            cout << "\033[1;33m" << candidate.sector_idx << "th flat_sector_candidate"
                << " / flatness: " << candidate.ground_flatness
                << " / line_variable: " << candidate.line_variable
                << " / ground_num : " << candidate.regionwise_ground.size()
                << "\033[0m" << endl;
        }

        double mu_flatness = mean_flatness + 1.5*stdev_flatness;
        double prob_flatness = 1/(1+exp( (candidate.ground_flatness-mu_flatness)/(mu_flatness/10) ));

        if (candidate.regionwise_ground.size() > 1500 && candidate.ground_flatness < th_dist_*th_dist_) prob_flatness = 1.0;

        double prob_line = 1.0;
        if (candidate.line_variable > 8.0 )//&& candidate.line_dir > M_PI/4)// candidate.ground_elevation > elevation_thr_[concentric_idx])
        {
            // if (verbose_) cout << "line_dir: " << candidate.line_dir << endl;
            prob_line = 0.0;
        }

        bool revert = prob_line*prob_flatness > 0.5;

        if ( concentric_idx < num_rings_of_interest_ )
        {
            if (revert)
            {
                if (verbose_)
                {
                    cout << "\033[1;32m" << "REVERT TRUE" << "\033[0m" << endl;
                }

                revert_pc_ += candidate.regionwise_ground;
                cloud_ground += candidate.regionwise_ground;
            }
            else
            {
                if (verbose_)
                {
                    cout << "\033[1;31m" << "FINAL REJECT" << "\033[0m" << endl;
                }
                reject_pc_ += candidate.regionwise_ground;
                cloud_nonground += candidate.regionwise_ground;
            }
        }
    }

    if (verbose_) std::cout << "\033[1;34m" << "====================================================" << "\033[0m" << endl;
}

// For adaptive
template<typename PointT>
void MOT<PointT>::extract_piecewiseground(
        const int zone_idx, const pcl::PointCloud<PointT> &src,
        pcl::PointCloud<PointT> &dst,
        pcl::PointCloud<PointT> &non_ground_dst) {

    // 0. Initialization
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();

    // 1. Region-wise Vertical Plane Fitting (R-VPF)
    // : removes potential vertical plane under the ground plane
    pcl::PointCloud<PointT> src_wo_verticals;
    src_wo_verticals = src;

    if (enable_RVPF_)
    {
        for (int i = 0; i < num_iter_; i++)
        {
            extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_, th_seeds_v_);
            estimate_plane(ground_pc_);

            if (zone_idx == 0 && normal_(2) < uprightness_thr_)
            {
                pcl::PointCloud<PointT> src_tmp;
                src_tmp = src_wo_verticals;
                src_wo_verticals.clear();

                Eigen::MatrixXf points(src_tmp.points.size(), 3);
                int j = 0;
                for (auto &p:src_tmp.points) {
                    points.row(j++) << p.x, p.y, p.z;
                }
                // ground plane model
                Eigen::VectorXf result = points * normal_;

                for (int r = 0; r < result.rows(); r++) {
                    if (result[r] < th_dist_v_ - d_ && result[r] > -th_dist_v_ - d_) {
                        non_ground_dst.points.push_back(src_tmp[r]);
                        vertical_pc_.points.push_back(src_tmp[r]);
                    } else {
                        src_wo_verticals.points.push_back(src_tmp[r]);
                    }
                }
            }
            else break;
        }
    }

    extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_);
    estimate_plane(ground_pc_);

    // 2. Region-wise Ground Plane Fitting (R-GPF)
    // : fits the ground plane

    //pointcloud to matrix
    Eigen::MatrixXf points(src_wo_verticals.points.size(), 3);
    int j = 0;
    for (auto &p:src_wo_verticals.points) {
        points.row(j++) << p.x, p.y, p.z;
    }

    for (int i = 0; i < num_iter_; i++) {

        ground_pc_.clear();

        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < th_dist_ - d_ ) {
                    ground_pc_.points.push_back(src_wo_verticals[r]);
                }
            } else { // Final stage
                if (result[r] < th_dist_ - d_ ) {
                    dst.points.push_back(src_wo_verticals[r]);
                } else {
                    non_ground_dst.points.push_back(src_wo_verticals[r]);
                }
            }
        }

        if (i < num_iter_ -1) estimate_plane(ground_pc_);
        else estimate_plane(dst);
    }
}

template<typename PointT>
void MOT<PointT>::calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev)
{
    if (vec.size() <= 1) return;

    mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

    for (int i=0; i<vec.size(); i++) { stdev += (vec.at(i)-mean)*(vec.at(i)-mean); }
    stdev /= vec.size()-1;
    stdev = sqrt(stdev);
}

template<typename PointT>
double MOT<PointT>::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    // if (y >= 0) {
    //     return atan2(y, x); // 1, 2 quadrant
    // } else {
    //     return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
    // }

    double angle = atan2(y, x);
    return angle > 0 ? angle : 2*M_PI+angle;
}

template<typename PointT>
double MOT<PointT>::xy2radius(const double &x, const double &y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

template<typename PointT>
void MOT<PointT>::callbackCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
{   
    RCLCPP_INFO(this->get_logger(),"Cloud Callback");
    is_frozen = true;
    double every_thing_start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
    double yaw = latest_vn_msg_.orientation_ypr.z;
    double pitch = latest_vn_msg_.orientation_ypr.y;
    double roll = latest_vn_msg_.orientation_ypr.x;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();
    Eigen::Vector3d translationVector;
    translationVector<<latest_vn_msg_.position_enu_ins.x, latest_vn_msg_.position_enu_ins.y,0;
    transformMatrix = Eigen::Matrix4d::Identity();
    transformMatrix.block<3,3>(0,0) = rotationMatrix;
    transformMatrix.block<3,1>(0,3) = translationVector;

    double time_taken;

    pcl::PointCloud<PointT> pc_read;
    pcl::PointCloud<PointT> pc_ground;
    pcl::PointCloud<PointT> pc_non_ground;
    pcl::PointCloud<PointT> pc_curbside;
    pcl::PointCloud<PointT> pc_object;
    pcl::PointCloud<PointT> pc_transformed;

    pcl::fromROSMsg(*cloud_msg, pc_read);

double grid_start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
    pcl::PointCloud<PointT> pc_curr;

    for (const auto& point : pc_read)
    {
        if (point.z>3||point.z<-1 || point.x>50 || point.x<-50 || point.y>30 || point.y<-30)
        {
            continue;
        }
        pc_transformed.push_back(point);
    }

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(pc_transformed.makeShared());
    sor.setLeafSize(xy_leaf_size_, xy_leaf_size_, z_leaf_size_);
    sor.filter(pc_curr);

    if(display_time_){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Grid Filter"), "\033[1;33m" << "Grid Filter time: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds() - grid_start << " sec)" << "\033[0m");
    }

    estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    if(display_time_){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Ground Segment"), "\033[1;33m" << "Input PointCloud: " << pc_read.size() << " -> Ground: " << pc_ground.size() <<  "/ NonGround: " << pc_non_ground.size()
            << " (running_time: " << time_taken << " sec)" << "\033[0m");
    }

    //extract_curbside(pc_non_ground,pc_curbside);
    double obj_start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
    object_detection(pc_non_ground, pc_object, cloud_msg->header.stamp);
    if(display_time_){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Object Detection"), "\033[1;33m" << "Object detection time: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds() - obj_start << " sec)" << "\033[0m");
    }

    double pub_start = rclcpp::Clock{RCL_STEADY_TIME}.now().seconds();
    //pub_curbside->publish(cloud2msg(pc_curbside,cloud_msg->header.stamp, cloud_msg->header.frame_id));
    if(objects_cloud_pub_){
        pub_object->publish(cloud2msg(pc_object,cloud_msg->header.stamp, cloud_msg->header.frame_id));
    }
    // RCLCPP_INFO(this->get_logger(),"Current time: %d seconds and %d nanoseconds", 
    //     cloud_msg->header.stamp.sec, 
    //     cloud_msg->header.stamp.nanosec % 1000000000);
    if(visualize_){
        pub_cloud->publish(cloud2msg(pc_transformed, cloud_msg->header.stamp, cloud_msg->header.frame_id));
        pub_ground->publish(cloud2msg(pc_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));}
    if(non_ground_pub_){
        pub_non_ground->publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    }
    if(display_time_){
        RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;35m" << "Total process time: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds() - every_thing_start << " sec)" << "\033[0m");
    }
    is_frozen = false;
    if(display_time_){
        RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;32m" << "Publish time: " << rclcpp::Clock{RCL_STEADY_TIME}.now().seconds() - pub_start << " sec)" << "\033[0m");
    }
}

template<typename PointT>
sensor_msgs::msg::PointCloud2 MOT<PointT>::cloud2msg(pcl::PointCloud<PointT> cloud, builtin_interfaces::msg::Time stamp, std::string frame_id) {
    sensor_msgs::msg::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp=stamp;
    cloud_ROS.header.frame_id = frame_id_;
    return cloud_ROS;
}

template<typename PointT>
void MOT<PointT>::pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm, pcl::PointCloud<PointT> &cloud_nonground) {

    for (int i=0; i<src.size(); i++) {
        if ((!noise_idxs_.empty()) &&(i == noise_idxs_.front())) {
            noise_idxs_.pop();
            continue;
        }

        PointT pt = src.points[i];

        double r = xy2radius(pt.x, pt.y);
        if ((r <= max_range_) && (r > min_range_)) {
            double theta = xy2theta(pt.x, pt.y);

            int zone_idx = 0;
            if ( r < min_ranges_[1] ) zone_idx = 0;
            else if ( r < min_ranges_[2] ) zone_idx = 1;
            else if ( r < min_ranges_[3] ) zone_idx = 2;
            else zone_idx = 3;

            int ring_idx = min(static_cast<long>(((r - min_ranges_[zone_idx]) / ring_sizes_[zone_idx])), num_rings_each_zone_[zone_idx] - 1);
            int sector_idx = min(static_cast<long>((theta / sector_sizes_[zone_idx])), num_sectors_each_zone_[zone_idx] - 1);

            czm[zone_idx][ring_idx][sector_idx].points.emplace_back(pt);
        }
        else {
            cloud_nonground.push_back(pt);
        }
    }

    if (verbose_) cout << "[ CZM ] Divides pointcloud into the concentric zone model" << endl;
}

template class MOT<pcl::PointXYZI>; 
