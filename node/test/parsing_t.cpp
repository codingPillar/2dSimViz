#include <gtest/gtest.h>

#include "parser.h"

using namespace parsing;

TEST(MESSAGE_PARSING, lists){
    char buffer[] =  "[10,  9,7, \n70]";
    std::vector<float> distances = parseList(buffer, sizeof(buffer) - 1);
    EXPECT_EQ(distances, std::vector<float>({10, 9, 7, 70}));
}