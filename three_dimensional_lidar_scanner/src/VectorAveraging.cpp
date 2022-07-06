#include <algorithm>
#include <map>
#include <vector>

#include "VectorAveraging.h"

int VectorAveraging::getMedian(const std::vector<float> &vec, int start,
                               int end) {
  int n = end - start;
  n = (n + 1) / 2 - 1;
  return n + start;
}

std::pair<float, float>
VectorAveraging::getOutlierLimits(const std::vector<float> &data) {
  float iQR;
  float iQROffset;
  float lowerLimit;
  float upperLimit;

  int midIndex = getMedian(data, 0, data.size());
  float lQ = data[getMedian(data, 0, midIndex)];
  float uQ = data[getMedian(data, midIndex + 1, data.size())];

  iQR = uQ - lQ;
  iQROffset = iQR * 1.5;
  upperLimit = uQ + iQROffset;
  lowerLimit = lQ - iQROffset;

  return {lowerLimit, upperLimit};
}

float VectorAveraging::findAverageOfVector(const std::vector<float> &data) {
  float sum = 0;
  float count = 0;
  double upperLimit;
  double lowerLimit;

  std::tie(lowerLimit, upperLimit) = VectorAveraging::getOutlierLimits(data);

  std::for_each(
      data.begin(), data.end(),
      [&upperLimit, &lowerLimit, &count, &sum](float const &measuredDistance) {
        if (lowerLimit <= measuredDistance && measuredDistance <= upperLimit) {
          sum += measuredDistance;
          ++count;
        }
      });

  return sum / count;
}