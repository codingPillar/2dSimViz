#ifndef MESSAGES_H
#define MESSAGES_H

#include <vector>

struct LidarData{
    float minAngle;
    float maxAngle;
    float angleStep;
    std::vector<float> distances;
};

struct OdomData{
    float x;
    float y;
    float angle;
};

#endif //MESSAGES_H