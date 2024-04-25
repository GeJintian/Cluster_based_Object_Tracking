#ifndef RADAR_TYPE_H
#define RADAR_TYPE_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <numeric>
#include <queue>
#include <mutex>
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <cmath>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>


struct RadarPointType
{
  PCL_ADD_POINT4D;
  union
  {
    struct
    {
      float velocity;
      float snr;
      float rcs;
      float confidence;
      float velocity_interval;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT (RadarPointType,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, velocity, velocity)
                                    (float, snr, snr)
                                    (float, rcs, rcs)
                                    (float, confidence, confidence)
                                    (float, velocity_interval, velocity_interval))

class ClusterProperties{
  public:
    double center_x;
    double center_y;
    double center_z;
    virtual double get_x() const { return center_x; }
    virtual double get_y() const { return center_y; }
};

class RadarClusterProperties : public ClusterProperties{
  public:
    double center_x;
    double center_y;
    double center_z;
    double yaw; //if possible
    double v_d;
    double v_x; //if possible
    double v_y; //if possible
    virtual double get_x() const { return center_x; }
    virtual double get_y() const { return center_y; }
    pcl::PointCloud<RadarPointType>::Ptr merged_cloud_{new pcl::PointCloud<RadarPointType>};
    RadarClusterProperties(double x, double y, double z, double vd, double yaw = 0, double vx = 0, double vy = 0):center_x(x), center_y(y), center_z(z), yaw(yaw), v_d(vd), v_x(vx), v_y(vy){}
};

class LidarClusterProperties : public ClusterProperties{
  public:
    double center_x;
    double center_y;
    double center_z;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    virtual double get_x() const { return center_x; }
    virtual double get_y() const { return center_y; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud_{new pcl::PointCloud<pcl::PointXYZI>};

    LidarClusterProperties(double x, double y, double z, double min_x, double max_x, double min_y, double max_y):center_x(x), center_y(y), center_z(z), min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y) {}
};

class ReturnClusterProperties : public ClusterProperties{
  public:
    double center_x;
    double center_y;
    double yaw; //if possible
    double v_d;
    virtual double get_x() const { return center_x; }
    virtual double get_y() const { return center_y; }
    ReturnClusterProperties(double x, double y, double vd = 0, double yaw = 0):center_x(x), center_y(y), yaw(yaw), v_d(vd){}
};

inline void radar2lidar_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar, pcl::PointCloud<RadarPointType>::Ptr cloud_radar){
  for (int idx =0; idx < cloud_radar->points.size(); idx ++) {
    pcl::PointXYZ point;
    point.x = cloud_radar->points[idx].x;
    point.y = cloud_radar->points[idx].y;
    point.z = 0;
    cloud_lidar->points.push_back(point);
}
}

inline int get_distance(ClusterProperties &a, ClusterProperties &b){
  return sqrt((a.get_x()-b.get_x())*(a.get_x()-b.get_x())+(a.get_y()-b.get_y())*(a.get_y()-b.get_y()));
}

#endif