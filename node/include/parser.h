#ifndef PARSER_H
#define PARSER_H

#include <vector>

#include "common.h"
#include "messages.h"

namespace parsing {

bool isWhiteSpace(char element);
std::pair<std::vector<float>, unsigned int> parseList(const char *buffer, unsigned int length);

struct LidarData parseLidarObj(const char *buffer, unsigned int length);
void parseHttpReq(const char *buffer, unsigned int length);

}

#endif //PARSER_H