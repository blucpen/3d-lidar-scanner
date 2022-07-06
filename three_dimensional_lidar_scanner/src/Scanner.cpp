#include "Scanner.h"
#include "three_dimensional_lidar_scanner/AverageScans.h"
#include "three_dimensional_lidar_scanner/Servo.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

Scanner::Scanner() : tfListener(tfBuffer) {

  nodeHandle.param<int>("number_of_scans_to_average",
                        numberOfScansToAveragePerAngle, 10);
  nodeHandle.param<int>("minimum_scan_angle", minimumScanAngle, 90);
  nodeHandle.param<int>("maximum_scan_angle", maximumScanAngle, 180);
  nodeHandle.param<std::string>("base_frame", baseFrame, "axle");
  nodeHandle.param<std::string>("laser_pointcloud", laserPointcloudTopic,
                                "laser_pointcloud");

  cloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(
      laserPointcloudTopic, 1000);

  averageScansClient =
      nodeHandle.serviceClient<three_dimensional_lidar_scanner::AverageScans>(
          "average_scans");
  averageScansClient.waitForExistence();

  servoClient =
      nodeHandle.serviceClient<three_dimensional_lidar_scanner::Servo>(
          "servo_control_server");
  servoClient.waitForExistence();
}

sensor_msgs::LaserScan Scanner::getAverageScan() {
  three_dimensional_lidar_scanner::AverageScans srv;
  srv.request.numberToAverage = numberOfScansToAveragePerAngle;

  ROS_INFO("Requesting average scan of %d", numberOfScansToAveragePerAngle);
  if (averageScansClient.call(srv)) {
    ROS_INFO("Received average scan");
    return srv.response.scan;
  } else {
    ROS_ERROR("Failed to call service %s",
              averageScansClient.getService().c_str());
  }
}

int Scanner::moveServo(int angle) {
  if (angle < minimumScanAngle || angle > maximumScanAngle) {
    return -1;
  }

  three_dimensional_lidar_scanner::Servo srv;
  srv.request.angle = angle;

  ROS_INFO("Moving servo to %d degrees", angle);
  if (servoClient.call(srv)) {
    ROS_INFO("Servo response received");
    return srv.response.result;
  } else {
    ROS_ERROR("Failed to call service %s", servoClient.getService().c_str());
    return -1;
  }
}

sensor_msgs::PointCloud2
Scanner::convertScanToPcl(const sensor_msgs::LaserScan &scan) {
  sensor_msgs::PointCloud2 cloud;
  projector.projectLaser(scan, cloud);
  return cloud;
}

sensor_msgs::PointCloud2 Scanner::transformPointcloudToBaseFrame(
    const sensor_msgs::PointCloud2 &inputCloud) {
  sensor_msgs::PointCloud2 outputCloud;
  pcl_ros::transformPointCloud(baseFrame, inputCloud, outputCloud, tfBuffer);
  return outputCloud;
}

void Scanner::scanEnvironment() {

  sensor_msgs::PointCloud2 outputCloud;

  for (int i = minimumScanAngle; i <= maximumScanAngle; ++i) {
    moveServo(i);

    const sensor_msgs::LaserScan &scan = getAverageScan();
    const sensor_msgs::PointCloud2 &scanPcl = convertScanToPcl(scan);
    const sensor_msgs::PointCloud2 &transformedPcl =
        transformPointcloudToBaseFrame(scanPcl);
    pcl::concatenatePointCloud(outputCloud, transformedPcl, outputCloud);

    cloudPublisher.publish(outputCloud);
  }

  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "three_dimensional_scanner");

  Scanner scanner;
  scanner.scanEnvironment();

  return 0;
}
