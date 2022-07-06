
#ifndef THREE_DIMENSIONAL_LIDAR_SCANNER_VECTORAVERAGING_H
#define THREE_DIMENSIONAL_LIDAR_SCANNER_VECTORAVERAGING_H

class VectorAveraging {
private:
  static int getMedian(const std::vector<float> &vec, int start, int end);

public:
  static float findAverageOfVector(const std::vector<float> &data);
  static std::pair<float, float>
  getOutlierLimits(const std::vector<float> &data);
};

#endif // THREE_DIMENSIONAL_LIDAR_SCANNER_VECTORAVERAGING_H
