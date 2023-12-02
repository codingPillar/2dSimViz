#ifndef PARSER_H
#define PARSER_H

#include <vector>

#include "common.h"

namespace parsing {

bool isWhiteSpace(char element);
std::vector<float> parseList(char *buffer, unsigned int length);
void parseHttpReq(char *buffer, unsigned int length);

}

#endif //PARSER_H