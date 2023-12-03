#ifndef PARSER_H
#define PARSER_H

#include <string>
#include <unordered_map>
#include <vector>

#include "common.h"
#include "messages.h"

#define MAX_ROUTE_STR_SIZE 128

namespace parsing {

enum HTTP_VERB{
    HTTP_GET, HTTP_POST, HTTP_NOT_SUPPORTED
};

struct HttpHeader{
    enum HTTP_VERB verb;
    char route[MAX_ROUTE_STR_SIZE];
    std::unordered_map<std::string, std::string> values;
    unsigned int bodyStartIndex;
};

bool isWhiteSpace(char element);
std::pair<std::vector<float>, unsigned int> parseList(const char *buffer, unsigned int length);

struct LidarData parseLidarObj(const char *buffer, unsigned int length);
HttpHeader parseHttpReq(const char *buffer, unsigned int length);

}

#endif //PARSER_H