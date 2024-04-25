#ifndef LIDAR_TYPE
#define LIDAR_TYPE

#include <pcl_conversions/pcl_conversions.h>

class LidarClusterProperties{
  public:
    double center_x;
    double center_y;
    double center_z;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    int obj_id;
    virtual double get_x() const { return center_x; }
    virtual double get_y() const { return center_y; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud_{new pcl::PointCloud<pcl::PointXYZI>};

    LidarClusterProperties(double x, double y, double z, double min_x, double max_x, double min_y, double max_y, int obj_id = -1):center_x(x), center_y(y), center_z(z), min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y), obj_id(obj_id) {}
};
#endif