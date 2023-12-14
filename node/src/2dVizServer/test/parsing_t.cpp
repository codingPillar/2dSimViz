#include <cstring>
#include <gtest/gtest.h>

#include "jsonParse.hpp"
#include "messages.h"
#include "parser.h"
#include "commonTest.h"

using namespace std;
using namespace parsing;

TEST(MESSAGE_PARSING, lidarDataObj){
    char buffer[] =  "{\"minAngle\": -1.0, \"maxAngle\": 1.0, \"angleStep\": 0.001, \"distances\":[10,9,7] }";
    const jsonParse::JsonObj obj = jsonParse::parseJson(buffer, sizeof(buffer));
    struct LidarData data = parseLidarObj(obj);
    EXPECT_TRUE(flteq(data.minAngle, -1.0F));
    EXPECT_TRUE(flteq(data.maxAngle, 1.0F));
    EXPECT_TRUE(flteq(data.angleStep, 0.001F));
    EXPECT_EQ(data.distances, std::vector<float>({10, 9, 7}));
}

TEST(MESSAGE_PARSING, OdomDataObj){
    char buffer[] =  "{\"x\": -1.0, \"y\": 1.0, \"angle\": 0.1}";
    const jsonParse::JsonObj obj = jsonParse::parseJson(buffer, sizeof(buffer));
    struct OdomData data = parseOdomObj(obj);
    EXPECT_TRUE(flteq(data.x, -1.0F));
    EXPECT_TRUE(flteq(data.y, 1.0F));
    EXPECT_TRUE(flteq(data.angle, 0.1F));
}

TEST(HTTP_PARSING, verb){
    string httpRequest = "GET / HTTP/1.1 \r\n accept-encoding: gzip, deflate, br \r\nAccept: */*\r\n\r\n";
    struct HttpHeader header = parseHttpReq(httpRequest.c_str(), httpRequest.length());
    EXPECT_EQ(header.verb, HTTP_GET);
}

TEST(HTTP_PARSING, route){
    string httpRequest = "GET / HTTP/1.1 \r\n accept-encoding: gzip, deflate, br \r\nAccept: */*\r\n\r\n";
    struct HttpHeader header = parseHttpReq(httpRequest.c_str(), httpRequest.length());
    EXPECT_TRUE(strcmp(header.route, "/") == 0);
}

TEST(HTTP_PARSING, headerSingle){
    string httpRequest = "GET / HTTP/1.1 \r\n accept-encoding: gzip, deflate, br \r\n\r\n";
    struct HttpHeader header = parseHttpReq(httpRequest.c_str(), httpRequest.length());
    auto encoding = header.values.find("accept-encoding");
    EXPECT_TRUE(encoding != header.values.end());
    EXPECT_TRUE(encoding->second == string("gzip, deflate, br "));
}

TEST(HTTP_PARSING, headerMultiple){
    string httpRequest = "GET / HTTP/1.1 \r\n accept-encoding: gzip, deflate, br \r\nAccept: */*\r\n\r\n";
    struct HttpHeader header = parseHttpReq(httpRequest.c_str(), httpRequest.length());
    auto encoding = header.values.find("accept-encoding");
    EXPECT_TRUE(encoding != header.values.end());
    EXPECT_TRUE(encoding->second == string("gzip, deflate, br "));
    auto accept = header.values.find("Accept");
    EXPECT_TRUE(accept != header.values.end());
    EXPECT_TRUE(accept->second == string("*/*"));
}

TEST(HTTP_PARSING, body){
    string httpRequest = "GET / HTTP/1.1 \r\n accept-encoding: gzip, deflate, br \r\nAccept: */*\r\n\r\n{adam:holla}";
    struct HttpHeader header = parseHttpReq(httpRequest.c_str(), httpRequest.length());
    EXPECT_EQ(header.bodyStartIndex, 70);
}