#include <cstdlib>
#include <iostream>
#include <cstring>

#include "parser.h"

#define MAX_FLT_STR_SIZE 64
#define MAX_KEY_VALUE_BUFFER_SIZE 128

using namespace std;

/* PRIVATE FUNCTIONS */
static void assignLidarDataField(struct LidarData &data, const char *key, void *value){
    if(strcmp(key, LIDAR_DATA_MIN_ANGLE_KEY) == 0) data.minAngle = *((float*) value);
    else if(strcmp(key, LIDAR_DATA_MAX_ANGLE_KEY) == 0) data.maxAngle = *((float*) value);
    else if(strcmp(key, LIDAR_DATA_ANGLE_STEP_KEY) == 0) data.angleStep = *((float*) value);
    else if(strcmp(key, LIDAR_DATA_DISTANCES_KEY) == 0) data.distances = *((vector<float>*) value);
}

static bool isFltNumeric(char value){
    return (value >= '0' && value <= '9') || value == '.' || value== '-';
}

namespace parsing {

static char whiteChars[] = {' ', '\n', '\r'};
bool isWhiteSpace(char element){
    for(unsigned int i = 0; i < sizeof(whiteChars); i++)
        if(element == whiteChars[i]) return true;
    return false;
}

pair<vector<float>, unsigned int> parseList(const char *buffer, unsigned int length){
    std::vector<float> elems;
    unsigned int start = 0;
    bool valueStarted = false;
    bool waitingSeperator = false;
    char fltBuffer[MAX_FLT_STR_SIZE] = {0};
    for(unsigned int i = 0; i < length; i++){
        if(isFltNumeric(buffer[i]) && !valueStarted){
            start = i;
            valueStarted = true;
        }else if(isWhiteSpace(buffer[i]) && valueStarted){
            /* READ VALUE */
            strncpy(fltBuffer, &buffer[start], i - start + 1);
            elems.push_back(atof(fltBuffer));
            valueStarted = false;
            waitingSeperator = true;
        }else if(waitingSeperator){
            /* WAITING FOR NEXT SEPERATOR */
            if(isWhiteSpace(buffer[i])) continue;
            else{
                cout << "WRONG FORMAT FOR LISTS, ENCOUTERED VALUE WHEN WAITONG FOR SEPERATOR" << endl;
                break;
            }
        }else if(buffer[i] == ',' || buffer[i] == ']'){
            if(valueStarted){
                /* READ VALUE */
                strncpy(fltBuffer, &buffer[start], i - start + 1);
                elems.push_back(atof(fltBuffer));
            }
            valueStarted = false;
            waitingSeperator = false;
            if(buffer[i] == ']') return {elems, i};
        }
    }
    return {elems, length};
}

/* SUPPORTS ONLY VALUES OF TYPE STRING AND LIST OF NUMBERS */
enum PARSE_OBJECT_STATE{
    WAITING_INIT,
    READING_KEY,
    WAITING_KEY_VALUE_SEP,
    READING_VALUE,
    WAITING_SEP,
    DONE
};
struct LidarData parseLidarObj(const char *buffer, unsigned int length){
    struct LidarData data = {};
    enum PARSE_OBJECT_STATE state = WAITING_INIT;
    bool reading = false;
    char keyBuffer[MAX_KEY_VALUE_BUFFER_SIZE] = {0};
    unsigned int start = 0;
    for(unsigned int i = 0; i < length; i++){
        switch (state) {
            case WAITING_INIT:{
                if(buffer[i] == '{') state = READING_KEY;
                else if(isWhiteSpace(buffer[i])) continue;
                else{
                    cout << "ERROR WHILE PARSING JSON, FIRST CHAR NOT {" << endl;
                    return data;
                }
            }break;
            case READING_KEY:{
                if(isWhiteSpace(buffer[i])) continue;
                else if(!reading && buffer[i] == '"'){
                    start = i;
                    reading = true;
                }else if(reading && buffer[i] == '"'){
                    strncpy(keyBuffer, &buffer[start + 1], i - start - 1);
                    keyBuffer[i - start - 1] = '\0';
                    state = WAITING_KEY_VALUE_SEP;
                    reading = false;
                }
            }break;
            case WAITING_KEY_VALUE_SEP:{
                if(isWhiteSpace(buffer[i])) continue;
                if(buffer[i] == ':') state = READING_VALUE;
                else{
                    cout << "ERROR WHILE PARSING, WAITING FOR : BUT ENCOUTERED " << buffer[i] << endl;
                    return {};
                }
            }break;
            case READING_VALUE:{
                /* FLOAT OR LIST */
                if(!reading && isWhiteSpace(buffer[i])) continue;
                if(!reading && isFltNumeric(buffer[i])){
                    start = i;
                    reading = true;
                }else if(!reading && buffer[i] == '['){
                    pair<vector<float>, unsigned int> list = parseList(&buffer[i], length - i);
                    assignLidarDataField(data, keyBuffer, (void*) &list.first);
                    i += list.second;
                    state = WAITING_SEP;
                    reading = false;
                }else if(reading && (isWhiteSpace(buffer[i]) || buffer[i] == ',' || buffer[i] == '}')){
                    /* DONE READING FLT */
                    float value = atof(&buffer[start]);
                    assignLidarDataField(data, keyBuffer, (void*)&value);
                    reading = false;
                    if(isWhiteSpace(buffer[i])) state = WAITING_SEP;
                    else if(buffer[i] == ',') state = READING_KEY;
                    else state = DONE;
                }
            }break;
            case WAITING_SEP:{
                reading = false;
                if(isWhiteSpace(buffer[i])) continue;
                else if(buffer[i] == ',') state = READING_KEY;
                else if(buffer[i] == '}') state = DONE;
                else{
                    cout << "WAITING FOR SEPERATOR BUT ENCOUTERED INVALID VALUE, ABORT" << endl;
                    return {};
                }
            }break;
            case DONE:{
                return data;
            }break;
        }
    }
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
            cout << "FOR NOW DONE WITH PARSING HEADER" << endl;
            header.bodyStartIndex = i;
            return header;
        }break;
        }
    }
    return header;
}

}