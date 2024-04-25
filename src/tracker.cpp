#include "detection/tracker.hpp"

// std::pair<int, int> findIndexOfMin(std::vector<std::vector<float>> distMat) {
//   //cout << "findIndexOfMin cALLED\n";
//   std::pair<int, int> minIndex;
//   float minEl = std::numeric_limits<float>::max();
//   //cout << "minEl=" << minEl << "\n";
//   for (int i = 0; i < distMat.size(); i++)
//     for (int j = 0; j < distMat.at(0).size(); j++) {
//       if (distMat[i][j] < minEl) {
//         minEl = distMat[i][j];
//         minIndex = std::make_pair(i, j);
//       }
//     }
//   //cout << "minIndex=" << minIndex.first << "," << minIndex.second << "\n";
//   return minIndex;
// }

SingleKFTracker::SingleKFTracker():stateSize(4), measSize(2), contrSize(0){
  //initialize
  KF.init(stateSize, measSize, contrSize, CV_32F);
  //state matrix
  KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 
    1, 0, DT, 0,  // for x: x_new = x_old + vx_old * dt (dt=1)
    0, 1, 0, DT,  // for y: y_new = y_old + vy_old * dt (dt=1)
    0, 0, 1, 0,  // for vx: vx_new = vx_old
    0, 0, 0, 1); // for vy: vy_new = vy_old
  //measurement matrix
  KF.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
  KF.measurementMatrix.at<float>(0, 0) = 1;
  KF.measurementMatrix.at<float>(1, 1) = 1;
  //Q
  KF.processNoiseCov = cv::Mat::eye(stateSize, stateSize, CV_32F) * 1e-2;
  //R
  KF.measurementNoiseCov = cv::Mat::eye(measSize, measSize, CV_32F) * 1e-1;
  //P
  KF.errorCovPost = cv::Mat::eye(stateSize, stateSize, CV_32F);
  // init
  KF.statePost = cv::Mat::zeros(stateSize, 1, CV_32F);
}

void SingleKFTracker::track(double x, double y){
  cv::Mat measurement = (cv::Mat_<float>(2,1) << x, y);//TODO consider if provide initial value (statePost)
  nowClusterCenter[0] = x;
  nowClusterCenter[1] = y;
  KF.correct(measurement);
  cv::Mat result = KF.predict();
  predClusterCenter[0] = result.at<float>(0);
  predClusterCenter[1] = result.at<float>(1);
}

void SingleKFTracker::track(){
  nowClusterCenter[0] = predClusterCenter[0];
  nowClusterCenter[1] = predClusterCenter[1];
  cv::Mat result = KF.predict();
  predClusterCenter[0] = result.at<float>(0);
  predClusterCenter[1] = result.at<float>(1);
}

SingleTracker::SingleTracker(){
  is_track = false;
  count_lose = 0;
  count_new = 0;
}

void SingleTracker::InitNewKFT(){
  tracker = SingleKFTracker();
}

void SingleTracker::track(double x, double y){
  std::cout<<"Single tracker activated"<<std::endl;
  count_lose = 0;
  if(count_new == 0 && !is_track) InitNewKFT();
  if(count_new > NEW_THRES) is_track = true;
  tracker.track(x,y);
  count_new += 1;
}

void SingleTracker::track(){
  std::cout<<"Single tracker activated with no measurement"<<std::endl;
  if(count_lose > LOSE_THRES){
    is_track = false;
    count_new = 0;
    return;
  }
  tracker.track();
  count_lose +=1;
}

std::vector<double> SingleTracker::report(){
  if(is_track){
    std::cout<<"Report position"<<std::endl;
    return tracker.nowClusterCenter;
  }
  std::cout<<"Don't report position"<<std::endl;
  std::vector<double> return_value;
  return return_value;
}

// void KFTracker::track(const std::vector<LidarClusterProperties> &property_set) {

//   // First predict, to update the internal statePre variable
//   std::vector<cv::Mat> pred;
//   for(int i = 0; i < kf_collection.size(); i++){
//     pred.push_back(kf_collection[i]->prediction());
//   }

//   // Get measurements
//   // Extract the position of the clusters from the multiArray. To check if the
//   // data coming in, check the .z (every third) coordinate and that will be 0.0
//   std::vector<geometry_msgs::msg::Point> clusterCenters; // clusterCenters

//   int i = 0;
//   for (std::vector<float>::const_iterator it = ccs.data.begin();
//        it != ccs.data.end(); it += 3) {
//     geometry_msgs::msg::Point pt;
//     pt.x = *it;
//     pt.y = *(it + 1);
//     pt.z = *(it + 2);

//     clusterCenters.push_back(pt);
//   }

//   //  cout<<"CLusterCenters Obtained"<<"\n";
//   std::vector<geometry_msgs::msg::Point> KFpredictions;
//   i = 0;
//   for (auto it = pred.begin(); it != pred.end(); it++) {
//     geometry_msgs::msg::Point pt;
//     pt.x = (*it).at<float>(0);
//     pt.y = (*it).at<float>(1);
//     pt.z = (*it).at<float>(2);

//     KFpredictions.push_back(pt);
//   }
//   // cout<<"Got predictions"<<"\n";

//   // Find the cluster that is more probable to be belonging to a given KF.
//   objID.clear();   // Clear the objID vector
//   objID.resize(6); // Allocate default elements so that [i] doesnt segfault.
//                    // Should be done better
//   // Copy clusterCentres for modifying it and preventing multiple assignments of
//   // the same ID
//   std::vector<geometry_msgs::msg::Point> copyOfClusterCenters(clusterCenters);
//   std::vector<std::vector<float>> distMat;

//   for (int filterN = 0; filterN < 6; filterN++) {
//     std::vector<float> distVec;
//     for (int n = 0; n < 6; n++) {
//       distVec.push_back(
//           euclidean_distance(KFpredictions[filterN], copyOfClusterCenters[n]));
//     }

//     distMat.push_back(distVec);
//     /*// Based on distVec instead of distMat (global min). Has problems with the
//     person's leg going out of scope int
//     ID=std::distance(distVec.begin(),min_element(distVec.begin(),distVec.end()));
//      //cout<<"finterlN="<<filterN<<"   minID="<<ID
//      objID.push_back(ID);
//     // Prevent assignment of the same object ID to multiple clusters
//      copyOfClusterCenters[ID].x=100000;// A large value so that this center is
//     not assigned to another cluster copyOfClusterCenters[ID].y=10000;
//      copyOfClusterCenters[ID].z=10000;
//     */
//     cout << "filterN=" << filterN << "\n";
//   }

//   cout << "distMat.size()" << distMat.size() << "\n";
//   cout << "distMat[0].size()" << distMat.at(0).size() << "\n";
//   // DEBUG: print the distMat
//   for (const auto &row : distMat) {
//     for (const auto &s : row)
//       std::cout << s << ' ';
//     std::cout << std::endl;
//   }

//   for (int clusterCount = 0; clusterCount < 6; clusterCount++) {
//     // 1. Find min(distMax)==> (i,j);
//     std::pair<int, int> minIndex(findIndexOfMin(distMat));
//     cout << "Received minIndex=" << minIndex.first << "," << minIndex.second
//          << "\n";
//     // 2. objID[i]=clusterCenters[j]; counter++
//     objID[minIndex.first] = minIndex.second;

//     // 3. distMat[i,:]=10000; distMat[:,j]=10000
//     distMat[minIndex.first] =
//         std::vector<float>(6, 10000.0); // Set the row to a high number.
//     for (int row = 0; row < distMat.size();
//          row++) // set the column to a high number
//     {
//       distMat[row][minIndex.second] = 10000.0;
//     }
//     // 4. if(counter<6) got to 1.
//     cout << "clusterCount=" << clusterCount << "\n";
//   }

//   // cout<<"Got object IDs"<<"\n";
//   // countIDs(objID);// for verif/corner cases

//   // display objIDs
//   /* DEBUG
//     cout<<"objID= ";
//     for(auto it=objID.begin();it!=objID.end();it++)
//         cout<<*it<<" ,";
//     cout<<"\n";
//     */

//   prevClusterCenters = clusterCenters;

//   markerPub->publish(clusterMarkers);

//   std_msgs::msg::Int32MultiArray obj_id;
//   for (auto it = objID.begin(); it != objID.end(); it++)
//     obj_id.data.push_back(*it);
//   // Publish the object IDs
//   objID_pub->publish(obj_id);
//   // convert clusterCenters from geometry_msgs::Point to floats
//   std::vector<std::vector<float>> cc;
//   for (int i = 0; i < 6; i++) {
//     vector<float> pt;
//     pt.push_back(clusterCenters[objID[i]].x);
//     pt.push_back(clusterCenters[objID[i]].y);
//     pt.push_back(clusterCenters[objID[i]].z);

//     cc.push_back(pt);
//   }
//   // cout<<"cc[5][0]="<<cc[5].at(0)<<"cc[5][1]="<<cc[5].at(1)<<"cc[5][2]="<<cc[5].at(2)<<"\n";
//   float meas0[2] = {cc[0].at(0), cc[0].at(1)};
//   float meas1[2] = {cc[1].at(0), cc[1].at(1)};
//   float meas2[2] = {cc[2].at(0), cc[2].at(1)};
//   float meas3[2] = {cc[3].at(0), cc[3].at(1)};
//   float meas4[2] = {cc[4].at(0), cc[4].at(1)};
//   float meas5[2] = {cc[5].at(0), cc[5].at(1)};

//   // The update phase
//   cv::Mat meas0Mat = cv::Mat(2, 1, CV_32F, meas0);
//   cv::Mat meas1Mat = cv::Mat(2, 1, CV_32F, meas1);
//   cv::Mat meas2Mat = cv::Mat(2, 1, CV_32F, meas2);
//   cv::Mat meas3Mat = cv::Mat(2, 1, CV_32F, meas3);
//   cv::Mat meas4Mat = cv::Mat(2, 1, CV_32F, meas4);
//   cv::Mat meas5Mat = cv::Mat(2, 1, CV_32F, meas5);

//   // cout<<"meas0Mat"<<meas0Mat<<"\n";
//   if (!(meas0Mat.at<float>(0, 0) == 0.0f || meas0Mat.at<float>(1, 0) == 0.0f))
//     Mat estimated0 = KF0.correct(meas0Mat);
//   if (!(meas1[0] == 0.0f || meas1[1] == 0.0f))
//     Mat estimated1 = KF1.correct(meas1Mat);
//   if (!(meas2[0] == 0.0f || meas2[1] == 0.0f))
//     Mat estimated2 = KF2.correct(meas2Mat);
//   if (!(meas3[0] == 0.0f || meas3[1] == 0.0f))
//     Mat estimated3 = KF3.correct(meas3Mat);
//   if (!(meas4[0] == 0.0f || meas4[1] == 0.0f))
//     Mat estimated4 = KF4.correct(meas4Mat);
//   if (!(meas5[0] == 0.0f || meas5[1] == 0.0f))
//     Mat estimated5 = KF5.correct(meas5Mat);

//   // Publish the point clouds belonging to each clusters

//   // cout<<"estimate="<<estimated.at<float>(0)<<","<<estimated.at<float>(1)<<"\n";
//   // Point statePt(estimated.at<float>(0),estimated.at<float>(1));
//   // cout<<"DONE KF_TRACKER\n";
// }

// void KFTracker::tracking_cb(){
//     if (firstFrame) {
//     // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.
//     // Could be made generic by creating a Kalman Filter only when a new object
//     // is detected

//     float dvx = 0.01f; // 1.0
//     float dvy = 0.01f; // 1.0
//     float dx = 1.0f;
//     float dy = 1.0f;
//     KF0.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
//                             dvx, 0, 0, 0, 0, dvy);
//     KF1.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
//                             dvx, 0, 0, 0, 0, dvy);
//     KF2.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
//                             dvx, 0, 0, 0, 0, dvy);
//     KF3.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
//                             dvx, 0, 0, 0, 0, dvy);
//     KF4.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
//                             dvx, 0, 0, 0, 0, dvy);
//     KF5.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
//                             dvx, 0, 0, 0, 0, dvy);

//     cv::setIdentity(KF0.measurementMatrix);
//     cv::setIdentity(KF1.measurementMatrix);
//     cv::setIdentity(KF2.measurementMatrix);
//     cv::setIdentity(KF3.measurementMatrix);
//     cv::setIdentity(KF4.measurementMatrix);
//     cv::setIdentity(KF5.measurementMatrix);
//     // Process Noise Covariance Matrix Q
//     // [ Ex 0  0    0 0    0 ]
//     // [ 0  Ey 0    0 0    0 ]
//     // [ 0  0  Ev_x 0 0    0 ]
//     // [ 0  0  0    1 Ev_y 0 ]
//     //// [ 0  0  0    0 1    Ew ]
//     //// [ 0  0  0    0 0    Eh ]
//     float sigmaP = 0.01;
//     float sigmaQ = 0.1;
//     setIdentity(KF0.processNoiseCov, Scalar::all(sigmaP));
//     setIdentity(KF1.processNoiseCov, Scalar::all(sigmaP));
//     setIdentity(KF2.processNoiseCov, Scalar::all(sigmaP));
//     setIdentity(KF3.processNoiseCov, Scalar::all(sigmaP));
//     setIdentity(KF4.processNoiseCov, Scalar::all(sigmaP));
//     setIdentity(KF5.processNoiseCov, Scalar::all(sigmaP));
//     // Meas noise cov matrix R
//     cv::setIdentity(KF0.measurementNoiseCov, cv::Scalar(sigmaQ)); // 1e-1
//     cv::setIdentity(KF1.measurementNoiseCov, cv::Scalar(sigmaQ));
//     cv::setIdentity(KF2.measurementNoiseCov, cv::Scalar(sigmaQ));
//     cv::setIdentity(KF3.measurementNoiseCov, cv::Scalar(sigmaQ));
//     cv::setIdentity(KF4.measurementNoiseCov, cv::Scalar(sigmaQ));
//     cv::setIdentity(KF5.measurementNoiseCov, cv::Scalar(sigmaQ));

//     // Process the point cloud
//     pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
//         new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
//         new pcl::PointCloud<pcl::PointXYZ>);
//     /* Creating the KdTree from input point cloud*/
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
//         new pcl::search::KdTree<pcl::PointXYZ>);

//     pcl::fromROSMsg(*input, *input_cloud);

//     tree->setInputCloud(input_cloud);

//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     ec.setClusterTolerance(0.08);
//     ec.setMinClusterSize(10);
//     ec.setMaxClusterSize(600);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(input_cloud);
//     /* Extract the clusters out of pc and save indices in cluster_indices.*/
//     ec.extract(cluster_indices);
//     return;

//     std::vector<pcl::PointIndices>::const_iterator it;
//     std::vector<int>::const_iterator pit;
//     // Vector of cluster pointclouds
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
//     // Cluster centroids
//     std::vector<pcl::PointXYZ> clusterCentroids;

//     for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
//           new pcl::PointCloud<pcl::PointXYZ>);
//       float x = 0.0;
//       float y = 0.0;
//       int numPts = 0;
//       for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

//         cloud_cluster->points.push_back(input_cloud->points[*pit]);
//         x += input_cloud->points[*pit].x;
//         y += input_cloud->points[*pit].y;
//         numPts++;

//         // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
//         //                                          origin);
//         // mindist_this_cluster = std::min(dist_this_point,
//         // mindist_this_cluster);
//       }

//       pcl::PointXYZ centroid;
//       centroid.x = x / numPts;
//       centroid.y = y / numPts;
//       centroid.z = 0.0;

//       cluster_vec.push_back(cloud_cluster);

//       // Get the centroid of the cluster
//       clusterCentroids.push_back(centroid);
//     }

//     // Ensure at least 6 clusters exist to publish (later clusters may be empty)
//     while (cluster_vec.size() < 6) {
//       pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
//           new pcl::PointCloud<pcl::PointXYZ>);
//       empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
//       cluster_vec.push_back(empty_cluster);
//     }

//     while (clusterCentroids.size() < 6) {
//       pcl::PointXYZ centroid;
//       centroid.x = 0.0;
//       centroid.y = 0.0;
//       centroid.z = 0.0;

//       clusterCentroids.push_back(centroid);
//     }

//     // Set initial state
//     KF0.statePre.at<float>(0) = clusterCentroids.at(0).x;
//     KF0.statePre.at<float>(1) = clusterCentroids.at(0).y;
//     KF0.statePre.at<float>(2) = 0; // initial v_x
//     KF0.statePre.at<float>(3) = 0; // initial v_y

//     // Set initial state
//     KF1.statePre.at<float>(0) = clusterCentroids.at(1).x;
//     KF1.statePre.at<float>(1) = clusterCentroids.at(1).y;
//     KF1.statePre.at<float>(2) = 0; // initial v_x
//     KF1.statePre.at<float>(3) = 0; // initial v_y

//     // Set initial state
//     KF2.statePre.at<float>(0) = clusterCentroids.at(2).x;
//     KF2.statePre.at<float>(1) = clusterCentroids.at(2).y;
//     KF2.statePre.at<float>(2) = 0; // initial v_x
//     KF2.statePre.at<float>(3) = 0; // initial v_y

//     // Set initial state
//     KF3.statePre.at<float>(0) = clusterCentroids.at(3).x;
//     KF3.statePre.at<float>(1) = clusterCentroids.at(3).y;
//     KF3.statePre.at<float>(2) = 0; // initial v_x
//     KF3.statePre.at<float>(3) = 0; // initial v_y

//     // Set initial state
//     KF4.statePre.at<float>(0) = clusterCentroids.at(4).x;
//     KF4.statePre.at<float>(1) = clusterCentroids.at(4).y;
//     KF4.statePre.at<float>(2) = 0; // initial v_x
//     KF4.statePre.at<float>(3) = 0; // initial v_y

//     // Set initial state
//     KF5.statePre.at<float>(0) = clusterCentroids.at(5).x;
//     KF5.statePre.at<float>(1) = clusterCentroids.at(5).y;
//     KF5.statePre.at<float>(2) = 0; // initial v_x
//     KF5.statePre.at<float>(3) = 0; // initial v_y

//     firstFrame = false;

//     for (int i = 0; i < 6; i++) {
//       geometry_msgs::msg::Point pt;
//       pt.x = clusterCentroids.at(i).x;
//       pt.y = clusterCentroids.at(i).y;
//       prevClusterCenters.push_back(pt);
//     }
//     /*  // Print the initial state of the kalman filter for debugging
//       cout<<"KF0.satePre="<<KF0.statePre.at<float>(0)<<","<<KF0.statePre.at<float>(1)<<"\n";
//       cout<<"KF1.satePre="<<KF1.statePre.at<float>(0)<<","<<KF1.statePre.at<float>(1)<<"\n";
//       cout<<"KF2.satePre="<<KF2.statePre.at<float>(0)<<","<<KF2.statePre.at<float>(1)<<"\n";
//       cout<<"KF3.satePre="<<KF3.statePre.at<float>(0)<<","<<KF3.statePre.at<float>(1)<<"\n";
//       cout<<"KF4.satePre="<<KF4.statePre.at<float>(0)<<","<<KF4.statePre.at<float>(1)<<"\n";
//       cout<<"KF5.satePre="<<KF5.statePre.at<float>(0)<<","<<KF5.statePre.at<float>(1)<<"\n";

//       //cin.ignore();// To be able to see the printed initial state of the
//       KalmanFilter
//       */
//   }

//   else {
//     // cout<<"ELSE firstFrame="<<firstFrame<<"\n";
//     pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
//         new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
//         new pcl::PointCloud<pcl::PointXYZ>);
//     /* Creating the KdTree from input point cloud*/
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
//         new pcl::search::KdTree<pcl::PointXYZ>);

//     pcl::fromROSMsg(*input, *input_cloud);

//     tree->setInputCloud(input_cloud);

//     /* Here we are creating a vector of PointIndices, which contains the actual
//      * index information in a vector<int>. The indices of each detected cluster
//      * are saved here. Cluster_indices is a vector containing one instance of
//      * PointIndices for each detected cluster. Cluster_indices[0] contain all
//      * indices of the first cluster in input point cloud.
//      */
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     ec.setClusterTolerance(0.3);
//     ec.setMinClusterSize(10);
//     ec.setMaxClusterSize(600);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(input_cloud);
//     // cout<<"PCL init successfull\n";
//     /* Extract the clusters out of pc and save indices in cluster_indices.*/
//     ec.extract(cluster_indices);
//     return;
//     // cout<<"PCL extract successfull\n";
//     /* To separate each cluster out of the vector<PointIndices> we have to
//      * iterate through cluster_indices, create a new PointCloud for each
//      * entry and write all points of the current cluster in the PointCloud.
//      */
//     // pcl::PointXYZ origin (0,0,0);
//     // float mindist_this_cluster = 1000;
//     // float dist_this_point = 1000;

//     std::vector<pcl::PointIndices>::const_iterator it;
//     std::vector<int>::const_iterator pit;
//     // Vector of cluster pointclouds
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;

//     // Cluster centroids
//     std::vector<pcl::PointXYZ> clusterCentroids;

//     for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
//       float x = 0.0;
//       float y = 0.0;
//       int numPts = 0;
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
//           new pcl::PointCloud<pcl::PointXYZ>);
//       for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

//         cloud_cluster->points.push_back(input_cloud->points[*pit]);

//         x += input_cloud->points[*pit].x;
//         y += input_cloud->points[*pit].y;
//         numPts++;

//         // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
//         //                                          origin);
//         // mindist_this_cluster = std::min(dist_this_point,
//         // mindist_this_cluster);
//       }

//       pcl::PointXYZ centroid;
//       centroid.x = x / numPts;
//       centroid.y = y / numPts;
//       centroid.z = 0.0;

//       cluster_vec.push_back(cloud_cluster);

//       // Get the centroid of the cluster
//       clusterCentroids.push_back(centroid);
//     }
//     // cout<<"cluster_vec got some clusters\n";

//     // Ensure at least 6 clusters exist to publish (later clusters may be empty)
//     while (cluster_vec.size() < 6) {
//       pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
//           new pcl::PointCloud<pcl::PointXYZ>);
//       empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
//       cluster_vec.push_back(empty_cluster);
//     }

//     while (clusterCentroids.size() < 6) {
//       pcl::PointXYZ centroid;
//       centroid.x = 0.0;
//       centroid.y = 0.0;
//       centroid.z = 0.0;

//       clusterCentroids.push_back(centroid);
//     }

//     std_msgs::msg::Float32MultiArray cc;
//     for (int i = 0; i < 6; i++) {
//       cc.data.push_back(clusterCentroids.at(i).x);
//       cc.data.push_back(clusterCentroids.at(i).y);
//       cc.data.push_back(clusterCentroids.at(i).z);
//     }
//     // cout<<"6 clusters initialized\n";

//     // cc_pos.publish(cc);// Publish cluster mid-points.
//     KFT(cc, objID_pub, markerPub);
//     int i = 0;
//     bool publishedCluster[6];
//     for (auto it = objID.begin(); it != objID.end();
//          it++) { // cout<<"Inside the for loop\n";

//       switch (i) {
//         //cout << "Inside the switch case\n";
//       case 0: {
//         publish_cloud(pub_cluster0, cluster_vec[*it]);
//         publishedCluster[i] =
//             true; // Use this flag to publish only once for a given obj ID
//         i++;
//         break;
//       }
//       case 1: {
//         publish_cloud(pub_cluster1, cluster_vec[*it]);
//         publishedCluster[i] =
//             true; // Use this flag to publish only once for a given obj ID
//         i++;
//         break;
//       }
//       case 2: {
//         publish_cloud(pub_cluster2, cluster_vec[*it]);
//         publishedCluster[i] =
//             true; // Use this flag to publish only once for a given obj ID
//         i++;
//         break;
//       }
//       case 3: {
//         publish_cloud(pub_cluster3, cluster_vec[*it]);
//         publishedCluster[i] =
//             true; // Use this flag to publish only once for a given obj ID
//         i++;
//         break;
//       }
//       case 4: {
//         publish_cloud(pub_cluster4, cluster_vec[*it]);
//         publishedCluster[i] =
//             true; // Use this flag to publish only once for a given obj ID
//         i++;
//         break;
//       }

//       case 5: {
//         publish_cloud(pub_cluster5, cluster_vec[*it]);
//         publishedCluster[i] =
//             true; // Use this flag to publish only once for a given obj ID
//         i++;
//         break;
//       }
//       default:
//         break;
//       }
//     }
//   }
// }