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