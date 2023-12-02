#include <iostream>
#include <cstring>

#include "parser.h"

using namespace std;

namespace parsing {

char whiteChars[] = {' ', '\n', '\r'};
bool isWhiteSpace(char element){
    for(unsigned int i = 0; i < sizeof(whiteChars); i++)
        if(element == whiteChars[i]) return true;
    return false;
}

vector<float> parseList(char *buffer, unsigned int length){
    std::vector<float> elems;
    unsigned int start = 0;
    bool encouteredChar = false;
    for(unsigned int i = 0; i < length; i++){
        if(isWhiteSpace(buffer[i])) continue;
        else if(!encouteredChar && buffer[i] != '['){
            cout << "[ERROR] WRONG FORMAT FOR JSON LIST" << endl;
            break;
        }else if(buffer[i] == ','){
            /* NEW ELEM TO BE PARSED */
            strtof(buffer, &buffer); 
        }else{
            /* START OF NEW ELEM */
            start = i;
        }
        encouteredChar = true;
    }
    /* TODO, REMOVE */
    (void) start;
    return elems;
}

/* WE ASSUME VALID HTTP REQUEST */
void parseHttpReq(char *buffer, unsigned int length){
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