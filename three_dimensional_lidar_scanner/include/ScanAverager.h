#ifndef SRC_SCANAVERAGER_H
#define SRC_SCANAVERAGER_H

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

class ScanAverager {
private:
  const int numberOfScansToAverage;
  const std::string topicName;

  int scansRead;
  bool isFirstScan;
  sensor_msgs::LaserScan msgTemplate;
  std::map<float, std::vector<float>> rangesByAngle;

  std::vector<float>
  findAverageOfScans(const std::map<float, std::vector<float>> &scans);
  void callBack(const sensor_msgs::LaserScan::ConstPtr &msg);

public:
  explicit ScanAverager(int numberOfScansToAverage, std::string laserTopicName);
  sensor_msgs::LaserScan getAverageScan();
};

#endif // SRC_SCANAVERAGER_H
