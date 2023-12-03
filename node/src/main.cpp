#include <iostream>
#include <vector>
#include <array>
#include <thread>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "common.h"
#include "messages.h"
#include "parser.h"

using namespace std;
using namespace parsing;

struct Args{
    uint16_t port;
} args; 

bool running = true;
struct LidarData lastLidarData;

char *readArg(int *argc, char ***argv){
    if(*argc == 0){
        cout << "USAGE\nnode <port>" << endl;
        exit(EXIT_ERROR);
    }
    char *temp = (*argv)[0];
    (*argv)++;
    (*argc)--;
    return temp;
}

#ifndef TEST_EXEC
int main(int argc, char **argv){
    cout << "WEB 2DVIZ SIMULATION SERVER START" << endl;

    /* READ ARGS*/
    (void) readArg(&argc, &argv); /* SKIP THE NAME ARG */
    args.port = (uint16_t) atoi(readArg(&argc, &argv));

    /* OPEN TCP/HTTP SERVER (FOR NOW WE HANDLE ONLY ONE CLIENT) */
    struct sockaddr_in addr = {};
    addr.sin_port = htons(args.port);
    addr.sin_family = AF_INET;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if(bind(fd, (const sockaddr *) &addr, sizeof(addr)) != 0){
        cout << "[ERROR] could not bind to port: " << args.port << endl;
        return EXIT_ERROR;
    }
    if(listen(fd, SERVER_REQ_BUFFER) != 0){
        cout << "[ERROR] COULD NOT START SERVER LISTEN" << endl;
    }
    cout << "SERVER LISTENING ON PORT: " << args.port << endl;

    /* START LISTENING ON SECOND THREAD */
    thread commThread{[fd](){
        struct sockaddr_in clientAddr;
        unsigned int size = sizeof(clientAddr);
        array<char, MAX_BUFFER_SIZE> buffer;
        while(running){
            int connectionFd = accept(fd, (struct sockaddr*) &clientAddr, &size);
            if(connectionFd < 0){
                cout << "[ERROR] COULD NOT ACCEPT CONNECTION, ABORT" << endl;
                continue;
            }
            int size = recv(connectionFd, buffer.data(), buffer.size(), 0);
            if(size <= 0){
                cout << "CLIENT DISCONNECTED, NEXT CONNECTION" << endl;
                continue;
            }
            cout << "RECEIVED MESSAGE OF SIZE: " << size << endl;
            HttpHeader header = parseHttpReq(buffer.data(), size);

            int httpCode = HTTP_OK_CODE;
            string httpCodeStr = "OK";
            string response;
            /* ROUTE PACKET BY LOOKING AT VERB AND ROUTE */
            if(strcmp(header.route, GET_CMD_VEL_VALUES) == 0 && header.verb == parsing::HTTP_GET){
                /* TODO, SEND CMD_VEL DATA */
            }else if(strcmp(header.route, POST_LIDAR_DATA_ROUTE) == 0 && header.verb == parsing::HTTP_POST){
                lastLidarData = parseLidarObj(&buffer.data()[header.bodyStartIndex], size - header.bodyStartIndex);
            }else{
                cout << "ROUTE: " << header.route << " NOT KNOWN, 404" << endl;
                httpCode = HTTP_ERROR_CODE;
                httpCodeStr = "";
            }

            /* GENERATE RESPONSE */
            int responseSize = sprintf(buffer.data(), 
                HTTP_RESPONSE_FORMAT HTTP_HEADER_FIELD_DELIM
                "Content-Length: 0" HTTP_HEADER_FIELD_DELIM
                HTTP_HEADER_FIELD_DELIM
                "%s", httpCode, httpCodeStr.c_str(), response.c_str());
            int sent = 0;
            do{ sent += send(connectionFd, buffer.data(), responseSize, 0); }
            while(sent != responseSize);  
            close(connectionFd);
        }
    }};

    /* PUBLISH ROS TOPICS FOR CMD_VEL AND SENSORS */

    /* HANDLE ROS EVENTS ON MAIN THREAD */
    while(running);

    return 0;
}
#endif //TEST_EXEC