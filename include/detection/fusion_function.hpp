#ifndef FUSION_FUNCTION_H
#define FUSION_FUNCTION_H


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
//#include <ceres/ceres.h>
#include <vector>
#include <cmath>
#include <limits>
#include "detection/radar_type.hpp"
//#include "detection/kalman_filter.hpp"

#define Z_UPPER_BOUND 3
#define SELF_BOUND 5
#define VELOCITY_BOUND 2
#define MAX_ASSOCIATION_DISTANCE 5
                                    

struct VelocityEstimatorResidual {
    VelocityEstimatorResidual(double azimuth, double vd) : azimuth_(azimuth), vd_(vd) {}

    template <typename T>
    bool operator()(const T* const v_ego, T* residual) const {
        residual[0] = v_ego[0] * cos(T(azimuth_)) + v_ego[1] * sin(T(azimuth_)) - T(vd_);
        return true;
    }

private:
    const double azimuth_, vd_;
};

//Eigen::Vector2d EstimateEgoVelocity(pcl::PointCloud<RadarPointType>::Ptr cloud, double init_x, double init_y);

void VAP(pcl::PointCloud<RadarPointType>::Ptr cloud, pcl::PointCloud<RadarPointType>::Ptr cloud_out, double init_x, double init_y);

double cluster_euclideanDistance(ClusterProperties &a, ClusterProperties &b);

double point_euclideanDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

void CLUSTER_ASSOCIATION_KNN(std::vector<LidarClusterProperties> &lidar_cluster_properties, std::vector<RadarClusterProperties> &radar_cluster_properties, std::vector<int> &associations);

void POINT_ASSOCIATION_KNN(pcl::PointCloud<pcl::PointXYZ>::Ptr radar_xyz, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_final);

#endif