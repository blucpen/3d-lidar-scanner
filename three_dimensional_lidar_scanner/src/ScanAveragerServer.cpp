#include "ScanAveragerServer.h"
#include "ScanAverager.h"
#include "ros/ros.h"

ScanAveragerServer::ScanAveragerServer() {
    nodeHandle.param<std::string>("laser_topic", laserTopic, "scan");
    ros::ServiceServer service = nodeHandle.advertiseService("average_scans", &ScanAveragerServer::serve, this);
    ROS_INFO("Server set up!");
    ros::spin();
}

bool ScanAveragerServer::serve(
    three_dimensional_lidar_scanner::AverageScans::Request &req,
    three_dimensional_lidar_scanner::AverageScans::Response &res) {

  ScanAverager scanAverager(req.numberToAverage, laserTopic);
  sensor_msgs::LaserScan averageScan = scanAverager.getAverageScan();
  res.scan = averageScan;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ScanAveragerServer");
  ScanAveragerServer scanAverager;
  return 0;
}
