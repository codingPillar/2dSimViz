#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <system_error>
#include <thread>
#include <vector>

#include "common.h"
#include "parser.h"

using namespace std;
using namespace parsing;

struct Args{
    uint16_t port;
} args; 

bool running = true;

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

    struct sockaddr_in clientAddr;
    unsigned int size = sizeof(clientAddr);
    int connectionFd = accept(fd, (struct sockaddr*) &clientAddr, &size);
    if(connectionFd < 0){
        cout << "[ERROR] COULD NOT ACCEPT CLIENT CONNECTION ABORT" << endl;
        return EXIT_ERROR;
    }

    /* START LISTENING ON SECOND THREAD */
    thread commThread{[connectionFd](){
        array<char, MAX_BUFFER_SIZE> buffer;
        while(running){
            int size = recv(connectionFd, buffer.data(), buffer.size(), 0);
            if(size <= 0){
                cout << "CLIENT DISCONNECTED, EXIT SERVER" << endl;
                exit(EXIT_ERROR);
            }
            cout << "RECEIVED MESSAGE OF SIZE: " << size << endl;
            /* PARSE SENSOR DATA AND PUBLISH ON TOPIC */
            cout << buffer.data() << endl;
            parseHttpReq(buffer.data(), size);

            /* GENERATE RESPONSE */
            int responseSize = sprintf(buffer.data(), 
                HTTP_RESPONSE_FORMAT HTTP_HEADER_FIELD_DELIM
                "Content-Length: 0" HTTP_HEADER_FIELD_DELIM
                HTTP_HEADER_FIELD_DELIM, 200, "OK");
            printf("%s", buffer.data());
            int sent = send(connectionFd, buffer.data(), responseSize - 1, 0);
            cout << "SENT REPLY OF SIZE: " << sent << endl;
        }
    }};

    /* PUBLISH ROS TOPICS FOR CMD_VEL AND SENSORS */

    /* HANDLE ROS EVENTS ON MAIN THREAD */
    while(running);

    return 0;
}
#endif //TEST_EXEC