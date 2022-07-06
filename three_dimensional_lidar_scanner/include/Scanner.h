#ifndef THREE_DIMENSIONAL_LIDAR_SCANNER_SCANNER_H
#define THREE_DIMENSIONAL_LIDAR_SCANNER_SCANNER_H

#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_listener.h>

class Scanner {

public:
  Scanner();
  void scanEnvironment();

private:
  int numberOfScansToAveragePerAngle;
  int minimumScanAngle;
  int maximumScanAngle;
  std::string baseFrame;
  std::string laserPointcloudTopic;

  ros::NodeHandle nodeHandle;
  ros::ServiceClient averageScansClient;
  ros::ServiceClient servoClient;
  ros::Publisher cloudPublisher;

  laser_geometry::LaserProjection projector;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  int moveServo(int angle);
  sensor_msgs::LaserScan getAverageScan();

  sensor_msgs::PointCloud2 convertScanToPcl(const sensor_msgs::LaserScan &scan);
  sensor_msgs::PointCloud2
  transformPointcloudToBaseFrame(const sensor_msgs::PointCloud2 &inputCloud);
};

#endif // THREE_DIMENSIONAL_LIDAR_SCANNER_SCANNER_H
