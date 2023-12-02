#ifndef PARSER_H
#define PARSER_H

#include <vector>

#include "common.h"

namespace parsing {

bool isWhiteSpace(char element);
std::vector<float> parseList(const char *buffer, unsigned int length);
void parseHttpReq(const char *buffer, unsigned int length);

}

#endif //PARSER_H