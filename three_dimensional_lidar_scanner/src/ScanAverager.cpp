
#include "ScanAverager.h"
#include "VectorAveraging.h"
#include "sensor_msgs/LaserScan.h"

#include <cstring>
#include <map>

ScanAverager::ScanAverager(int numberOfScansToAverage,
                           std::string laserTopicName)
    : numberOfScansToAverage(numberOfScansToAverage),
      topicName(laserTopicName) {}

std::vector<float> ScanAverager::findAverageOfScans(
    const std::map<float, std::vector<float>> &scans) {
  std::vector<float> averageRanges;

  for (auto const &entry : scans) {
    std::vector<float> ranges = entry.second;
    std::sort(ranges.begin(), ranges.end());

    float average = VectorAveraging::findAverageOfVector(ranges);
    averageRanges.push_back(average);
  }

  return averageRanges;
}

void ScanAverager::callBack(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // Throwing away garbage first scan
  if (isFirstScan) {
    isFirstScan = false;
    return;
  }

  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    float current_angle = i * msg->angle_increment + msg->angle_min;
    float current_range = msg->ranges[i];
    rangesByAngle[current_angle].push_back(current_range);
  }

  ++scansRead;

  if (scansRead > numberOfScansToAverage) {
    msgTemplate = *msg;
  }
}

sensor_msgs::LaserScan ScanAverager::getAverageScan() {
  const int QUEUE_SIZE = 1;

  isFirstScan = true;
  scansRead = 0;

  ros::NodeHandle node;
  ros::Subscriber subsciber =
      node.subscribe(topicName, QUEUE_SIZE, &ScanAverager::callBack, this);

  while (scansRead <= numberOfScansToAverage) {
    ros::spinOnce();
  }

  subsciber.shutdown();

  sensor_msgs::LaserScan averageScan = msgTemplate;
  std::vector<float> averageRanges = findAverageOfScans(rangesByAngle);
  averageScan.ranges = averageRanges;

  return averageScan;
}
