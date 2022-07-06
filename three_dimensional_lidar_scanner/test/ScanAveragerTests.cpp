#include <gtest/gtest.h>
#include "VectorAveraging.h"
#include "ScanAverager.h"
#include "sensor_msgs/LaserScan.h"

TEST(VectorAveraging, outlierLimits) {
    float lowerLimit;
    float upperLimit;
    float EXPECTED_LOWER_LIMIT = -14.5;
    float EXPECTED_UPPER_LIMIT = 37.5;

    std::vector<float> data = {1, 2, 5, 6, 7, 9, 12, 15, 18, 19, 38};

    std::tie(lowerLimit, upperLimit) = VectorAveraging::getOutlierLimits(data);
    EXPECT_FLOAT_EQ(lowerLimit, EXPECTED_LOWER_LIMIT);
    EXPECT_FLOAT_EQ(upperLimit, EXPECTED_UPPER_LIMIT);
}

TEST(VectorAveraging, averageOfVector) {
    float EXPECTED_AVERAGE = 9.4;
    std::vector<float> data = {1, 2, 5, 6, 7, 9, 12, 15, 18, 19, 38};

    float average = VectorAveraging::findAverageOfVector(data);
    EXPECT_FLOAT_EQ(average, EXPECTED_AVERAGE);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}