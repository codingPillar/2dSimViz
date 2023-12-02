#include <gtest/gtest.h>

#include "messages.h"
#include "parser.h"
#include "commonTest.h"

using namespace parsing;

TEST(MESSAGE_PARSING, lists){
    char buffer[] =  "[10,  9,7, \n70]";
    std::vector<float> distances = parseList(buffer, sizeof(buffer) - 1).first;
    EXPECT_EQ(distances, std::vector<float>({10, 9, 7, 70}));
}

TEST(MESSAGE_PARSING, lidarDataObj){
    char buffer[] =  "{\"minAngle\": \"-1.0\", \"maxAngle\": \"1.0\", \"angleStep\": \"0.001\", \"distances\":[10,9,7]}";
    struct LidarData data = parseLidarObj(buffer, sizeof(buffer) - 1);
    EXPECT_TRUE(flteq(data.minAngle, -1.0F));
    EXPECT_TRUE(flteq(data.maxAngle, 1.0F));
    EXPECT_TRUE(flteq(data.angleStep, 0.001F));
    EXPECT_EQ(data.distances, std::vector<float>({10, 9, 7}));
}