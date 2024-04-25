#include "detection/fusion_function.hpp"


double cluster_euclideanDistance(ClusterProperties &a, ClusterProperties &b) {
    return get_distance(a,b);
}

double point_euclideanDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
    return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
}

void CLUSTER_ASSOCIATION_KNN(std::vector<LidarClusterProperties> &lidar_cluster_properties, std::vector<RadarClusterProperties> &radar_cluster_properties, std::vector<int> &associations){
    for (size_t i = 0; i < radar_cluster_properties.size(); ++i) {
        double minDistance = std::numeric_limits<double>::max();
        int minIndex = -1;
        for (size_t j = 0; j < lidar_cluster_properties.size(); ++j) {
            double distance = cluster_euclideanDistance(radar_cluster_properties[i], lidar_cluster_properties[j]);
            //std::cout<<"report distance "<<distance<<std::endl;
            if (distance < minDistance) {
                minDistance = distance;
                minIndex = j;
            }
        }
        if(minDistance < MAX_ASSOCIATION_DISTANCE){
            associations[i] = minIndex;  // Associate the index of the closest lidar measurement
        }
        //else std::cout<<"out of range "<<minDistance<<std::endl;
    }
}

void POINT_ASSOCIATION_KNN(pcl::PointCloud<pcl::PointXYZ>::Ptr radar_xyz, pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_final){}

// Eigen::Vector2d EstimateEgoVelocity(pcl::PointCloud<RadarPointType>::Ptr cloud, double init_x, double init_y) {
//     Eigen::VectorXd x(cloud->points.size());
//     Eigen::VectorXd y(cloud->points.size());
//     Eigen::VectorXd vd(cloud->points.size());

//     for (size_t i = 0; i < cloud->points.size(); ++i) {
//         x(i) = cloud->points[i].x;
//         y(i) = cloud->points[i].y;
//         vd(i) = cloud->points[i].velocity;
//     }

//     Eigen::VectorXd azimuth = Eigen::atan2(y.array(), x.array());
//     Eigen::Vector2d v_ego(init_x, init_y);

//     ceres::Problem problem;
//     for (size_t i = 0; i < cloud->points.size(); ++i) {
//         ceres::CostFunction* cost_function =
//             new ceres::AutoDiffCostFunction<VelocityEstimatorResidual, 1, 2>(
//                 new VelocityEstimatorResidual(azimuth(i), vd(i)));
//         problem.AddResidualBlock(cost_function, nullptr /* squared loss */, v_ego.data());
//     }

//     ceres::Solver::Options options;
//     options.linear_solver_type = ceres::DENSE_QR;
//     options.minimizer_progress_to_stdout = true;
//     ceres::Solver::Summary summary;
//     ceres::Solve(options, &problem, &summary);

//     // std::cout << summary.BriefReport() << "\n";
//     // std::cout << "Estimated v_ego: " << v_ego.transpose() << std::endl;

//     return v_ego;
// }

void VAP(pcl::PointCloud<RadarPointType>::Ptr cloud, pcl::PointCloud<RadarPointType>::Ptr cloud_out, double init_x, double init_y){
  //Eigen::Vector2d v_ego = EstimateEgoVelocity(cloud, init_x, init_y);
  Eigen::Vector2d v_ego;
  v_ego <<init_x, init_y;

  std::vector<int> outliers_indices;
  std::vector<double> residuals;

  for (size_t i = 0; i < cloud->points.size(); ++i) {
      double azimuth = std::atan2(cloud->points[i].y, cloud->points[i].x);
      double predicted_vd = v_ego[0] * std::cos(azimuth) + v_ego[1] * std::sin(azimuth);
      double residual = predicted_vd - cloud->points[i].velocity;
      residuals.push_back(residual);
  }

  double mean_residual = std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size();
  double sq_sum = std::inner_product(residuals.begin(), residuals.end(), residuals.begin(), 0.0);
  double std_dev = std::sqrt(sq_sum / residuals.size() - mean_residual * mean_residual);

  for (size_t i = 0; i < residuals.size(); ++i) {
      if (std::abs(residuals[i]) > std_dev) {
          outliers_indices.push_back(i);
      }
  }

  for (int idx : outliers_indices) {
      cloud_out->points.push_back(cloud->points[idx]);
  }
  //return v_ego;
}

