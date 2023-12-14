#ifndef PARSER_H
#define PARSER_H

#include <string>
#include <unordered_map>
#include <vector>

#include "common.h"
#include "messages.h"

#include "jsonParse.hpp"

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

struct LidarData parseLidarObj(const struct jsonParse::JsonObj &obj);
struct OdomData parseOdomObj(const struct jsonParse::JsonObj &obj);

HttpHeader parseHttpReq(const char *buffer, unsigned int length);

}

#endif //PARSER_H