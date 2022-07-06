#ifndef SRC_SCANAVERAGERSERVER_H
#define SRC_SCANAVERAGERSERVER_H

#include "three_dimensional_lidar_scanner/AverageScans.h"
#include <ros/ros.h>

class ScanAveragerServer {
private:
  ros::NodeHandle nodeHandle;
  std::string laserTopic;

public:
  ScanAveragerServer();
  bool serve(three_dimensional_lidar_scanner::AverageScans::Request &req,
             three_dimensional_lidar_scanner::AverageScans::Response &res);
};

#endif // SRC_SCANAVERAGERSERVER_H
