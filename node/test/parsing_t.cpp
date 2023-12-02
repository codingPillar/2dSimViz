#include <gtest/gtest.h>

#include "parser.h"

using namespace parsing;

TEST(MESSAGE_PARSING, lists){
    char buffer[] =  "[10,  9,7, \n70]";
    std::vector<float> distances = parseList(buffer, sizeof(buffer) - 1).first;
    EXPECT_EQ(distances, std::vector<float>({10, 9, 7, 70}));
}

TEST(MESSAGE_PARSING, lidarDataObj){
    char buffer[] =  "{minAngle: -1.0, maxAngle: 1.0, angleStep: 0.001, distances:[10,9,7]}";
    std::vector<float> distances = parseList(buffer, sizeof(buffer) - 1).first;
    EXPECT_EQ(distances, std::vector<float>({10, 9, 7, 70}));
}