#ifndef EKFTRACKING_H
#define EKFTRACKING_H

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "detection/lidar_type.hpp"

#define NEW_THRES 3
#define LOSE_THRES 3
#define DT 0.05

class SingleKFTracker{
public:
    int stateSize;
    int measSize;
    int contrSize;
    std::vector<double> nowClusterCenter{0,0}; // 
    std::vector<double> predClusterCenter{0,0}; //predict pos for next timestep
    // vector<int> predClusterSpeed;
    bool firstFrame = true;
    cv::KalmanFilter KF;

    SingleKFTracker();
    void track(double x, double y);
    void track();
};

class SingleTracker{
public:
    SingleKFTracker tracker;
    bool is_track;
    int count_lose;
    int count_new;

    SingleTracker();
    void InitNewKFT();
    void track(double x, double y);
    void track();
    std::vector<double> report();
};

// class KFTracker{
// public:
//     KFTracker();
//     void track(const std::vector<LidarClusterProperties> &property_set);
//     // void tracking_cb();
//     std::vector<LidarClusterProperties> prevClusterCenters;
//     bool firstFrame = true;
//     std::vector<int> objID;
//     std::vector<cv::KalmanFilter> kf_collection;
// };

//std::pair<int, int> findIndexOfMin(std::vector<std::vector<float>> distMat);
#endif