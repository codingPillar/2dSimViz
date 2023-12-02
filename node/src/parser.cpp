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
    return (value >= '0' && value <= '9') || value == '.';
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
                    strncpy(keyBuffer, &buffer[start], i - start + 2);
                    printf("KEY: %s\n", keyBuffer);
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
                /* STRING OR LIST */
                if(isWhiteSpace(buffer[i])) continue;
                if(!reading && buffer[i] == '"'){
                    start = i;
                    reading = true;
                }else if(!reading && buffer[i] == '['){
                    pair<vector<float>, unsigned int> list = parseList(&buffer[i], length - i);
                    assignLidarDataField(data, keyBuffer, (void*) &list.first);
                    i += list.second;
                    state = WAITING_SEP;
                }else if(reading && buffer[i] == '"' && buffer[start] == '"'){
                    /* DONE READING STRING */
                    float value = atof(&buffer[start]);
                    assignLidarDataField(data, keyBuffer, &value);
                    printf("KEY: %s, VALUE: %f", keyBuffer, value);
                    state = WAITING_SEP;
                }
            }break;
            case WAITING_SEP:{
                if(isWhiteSpace(buffer[i])) continue;
                else if(buffer[i] == ',') start = READING_KEY;
                else if(buffer[i] == '}') state = DONE;
            }break;
            case DONE:{
                return data;
            }break;
        }
    }
    return data;
}

/* WE ASSUME VALID HTTP REQUEST */
void parseHttpReq(const char *buffer, unsigned int length){
    /* WE WANT TO LOOK FOR END OF HEADER BY FINDING SEQUENCE \n\r\n\r */
    unsigned int lpointer = 0;
    for(; lpointer < length; lpointer++)
        if(memcmp(&buffer[lpointer], HTTP_END_HEADER_SEQ, sizeof(HTTP_END_HEADER_SEQ) - 1) == 0) break;
    /* NOT VALID HTTP STRING */
    if(lpointer == length) return;
    lpointer += sizeof(HTTP_END_HEADER_SEQ) - 1;
    /* LPOINTER IS NOW POINTING TO BODY SECTION, WE ASSUME FOR NOW THAT EVERYTHING IS SENT ON SAME REQUETS */
    printf("BODY: %s\n", &buffer[lpointer]);
}

}