#ifndef MESSAGES_H
#define MESSAGES_H

#include <vector>

struct LidarData{
    float minAngle;
    float maxAngle;
    float angleStep;
    std::vector<float> distances;
};

#endif //MESSAGES_H