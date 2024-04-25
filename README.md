## Lidar Object Tracking for YASMARINA Racing Car Circuit
ROS2 package for a simple Object tracking algorithm. Ground Segmentation is based on [patchworkpp](https://github.com/url-kaist/patchwork-plusplus).
The detection and tracking range of this algorithm is within the range of the circuit boundary. This limitation could be overcome, if Radar information is available in this [branch](https://github.com/GeJintian/Cluster_based_Object_Tracking/tree/radar_fusion).
Now only SOT is available, but a small change in tracker.hpp could enable MOT.
