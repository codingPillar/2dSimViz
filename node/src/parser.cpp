#include <cstdlib>
#include <iostream>
#include <cstring>

#include "parser.h"

#define MAX_FLT_STR_SIZE 64

using namespace std;

namespace parsing {

bool isFltNumeric(char value){
    return (value >= '0' && value <= '9') || value == '.';
}

char whiteChars[] = {' ', '\n', '\r'};
bool isWhiteSpace(char element){
    for(unsigned int i = 0; i < sizeof(whiteChars); i++)
        if(element == whiteChars[i]) return true;
    return false;
}

vector<float> parseList(const char *buffer, unsigned int length){
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
        }
    }
    return elems;
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