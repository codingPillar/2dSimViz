#include <cstdlib>
#include <iostream>
#include <cstring>
#include <iterator>
#include <string>
#include <vector>

#include "jsonParse.hpp"
#include "messages.h"
#include "parser.h"

#define MAX_FLT_STR_SIZE 64
#define MAX_KEY_VALUE_BUFFER_SIZE 128

using namespace std;

/* PRIVATE FUNCTIONS */
char whiteSpaces[] = {' ', '\n', '\r'};
static bool isWhiteSpace(char value){
    for(unsigned int i = 0; i < sizeof(whiteSpaces); i++)
        if(value == whiteSpaces[i]) return true;
    return false;
}

static void assignLidarDataField(struct LidarData &data, const char *key, void *value){
    if(strcmp(key, LIDAR_DATA_MIN_ANGLE_KEY) == 0) data.minAngle = *((float*) value);
    else if(strcmp(key, LIDAR_DATA_MAX_ANGLE_KEY) == 0) data.maxAngle = *((float*) value);
    else if(strcmp(key, LIDAR_DATA_ANGLE_STEP_KEY) == 0) data.angleStep = *((float*) value);
    else if(strcmp(key, LIDAR_DATA_DISTANCES_KEY) == 0) data.distances = *((vector<float>*) value);
}

namespace parsing {

struct LidarData parseLidarObj(const struct jsonParse::JsonObj &obj){
    const char* keys[] = {LIDAR_DATA_MIN_ANGLE_KEY, LIDAR_DATA_MAX_ANGLE_KEY, LIDAR_DATA_ANGLE_STEP_KEY, LIDAR_DATA_DISTANCES_KEY};
    struct LidarData lidarData;
    for(unsigned int i = 0; i < sizeof(keys) / sizeof(const char*); i++){
        const auto &iter = obj.values.find(std::string(keys[i]));
        if(iter == obj.values.end()){
            cout << "COULD NOT FIND KEY: " << keys[i] << " FOR LIDAR OBJECT" << endl;
            return {};
        }
        if(iter->second.type == jsonParse::JSON_NUMBER){
            assignLidarDataField(lidarData, keys[i], iter->second.value);
            continue;
        } 
        /* PARSE LIST */
        std::vector<float> ranges;
        auto &values = *(std::vector<jsonParse::JsonObj::JsonValue>*)iter->second.value;
        for(unsigned int i = 0; i < values.size(); i++) ranges.push_back(*((float*)values[i].value));
        assignLidarDataField(lidarData, keys[i], &ranges);
    }
    return lidarData;
}

struct OdomData parseOdomObj(const struct jsonParse::JsonObj &obj){
    const char* keys[] = {ODOM_DATA_POS_X_KEY, ODOM_DATA_POS_Y_KEY, ODOM_DATA_POS_ANGLE_KEY};
    struct OdomData data;
    for(unsigned int i = 0; i < sizeof(keys) / sizeof(char*); i++){
        const auto &iter = obj.values.find(std::string(keys[i]));
        if(iter == obj.values.end() || iter->second.type != jsonParse::JSON_NUMBER){
            cout << "COULD NOT FIND VALUE WITH KEY: " << keys[i] << " OR INVALID FORMAT" << endl;
            return data;
        } 
    }
    data.x = *((float*)obj.values.find(std::string(ODOM_DATA_POS_X_KEY))->second.value);
    data.y = *((float*)obj.values.find(std::string(ODOM_DATA_POS_Y_KEY))->second.value);
    data.angle = *((float*)obj.values.find(std::string(ODOM_DATA_POS_ANGLE_KEY))->second.value);
    return data;
}

enum HTTP_REQUEST_PARSE_STATE{
    PARSING_VERB,
    PARSING_ROUTE,
    PARSING_VERSION,
    PARSING_HEADER_KEY,
    WAITING_SEPARATOR,
    PARSING_HEADER_VALUE,
    PARSING_BODY
};
HttpHeader parseHttpReq(const char *buffer, unsigned int length){
    struct HttpHeader header;
    enum HTTP_REQUEST_PARSE_STATE state = PARSING_VERB;
    bool reading = false;
    unsigned int start = 0;
    string currentKey;
    string currentValue;
    for(unsigned int i = 0; i < length; i++){
        switch (state) {
        case PARSING_VERB:{
            if(!reading && isWhiteSpace(buffer[i])) continue;
            else if(!reading){
                reading = true;
                start = i;
            }else if(reading && isWhiteSpace(buffer[i])){
                /* READ VERB */
                if(strncmp(&buffer[start], "GET", i - start) == 0) header.verb = HTTP_GET;
                else if(strncmp(&buffer[start], "POST", i - start) == 0) header.verb = HTTP_POST;
                else{
                    cout << "UNSUPPORTED HTTP VERB ABORT" << endl;
                    header.verb = HTTP_NOT_SUPPORTED;
                    return header;
                }
                state = PARSING_ROUTE;
                reading = false;
            }
        }break;
        case PARSING_ROUTE:{
            if(!reading && isWhiteSpace(buffer[i])) continue;
            else if(!reading){
                reading = true;
                start = i;
            }else if(reading && isWhiteSpace(buffer[i])){
                strncpy(header.route, &buffer[start], i - start);
                header.route[i - start] = '\0';
                reading = false;
                state = PARSING_VERSION;
            }
        }break;
        case PARSING_VERSION:{
            if(strncmp(&buffer[i], HTTP_HEADER_FIELD_DELIM, sizeof(HTTP_HEADER_FIELD_DELIM) - 1) == 0)
                state = PARSING_HEADER_KEY;
        }break;
        case PARSING_HEADER_KEY:{
            if(!reading && isWhiteSpace(buffer[i])) continue;
            if(!reading){
                start = i;
                reading = true;
                currentKey = buffer[i];
            }
            else if(reading && buffer[i] == ':'){
                reading = false;
                state = PARSING_HEADER_VALUE;
            }
            else if(reading && !isWhiteSpace(buffer[i])) currentKey += buffer[i];
            else if(reading){
                reading = false;
                state = WAITING_SEPARATOR;
            }
        }break;
        case WAITING_SEPARATOR:{
            if(buffer[i] != ':') continue;
            state = PARSING_HEADER_VALUE;
        }break;
        case PARSING_HEADER_VALUE:{
            if(!reading && isWhiteSpace(buffer[i])) continue;
            if(!reading){
                reading = true;
                start = i;
                currentValue = buffer[i];
            }else if(reading && buffer[i] != '\r') currentValue += buffer[i];
            else{
                header.values.insert({currentKey, currentValue});
                reading = false;
                /* WE ASSUME ANY \r IS FOLLOWED BY \n */
                if(strncmp(&buffer[i], HTTP_END_HEADER_SEQ, sizeof(HTTP_END_HEADER_SEQ) - 1) == 0) state = PARSING_BODY;
                else state = PARSING_HEADER_KEY;
            } 
        }break;
        case PARSING_BODY:{
            if(!reading && isWhiteSpace(buffer[i])) continue;
            //cout << "FOR NOW DONE WITH PARSING HEADER" << endl;
            header.bodyStartIndex = i;
            return header;
        }break;
        }
    }
    return header;
}

}