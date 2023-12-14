#include <cmath>
#include <gtest/gtest.h>
#include <vector>

using namespace std;

/* IMPLEMENTATION FOR COMMON FUNCTIONS */
#define FLT_PRECISION 0.00001F
bool flteq(float a, float b){
    return abs(b - a) < FLT_PRECISION;
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}