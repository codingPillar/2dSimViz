#include <iostream>
#include <string>
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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#define NODE_NAME "SimViz2dServer"
#define COMMAND_VEL_TOPIC "cmd_vel"
#define LIDAR_DATA_TOPIC "limo/scan"
#define PUBLISHER_QUEUE_SIZE 32
#define PUBLISH_RATE 20

using namespace std;
using namespace parsing;

struct Args{
    uint16_t port;
} args; 

bool running = true;
/* MAYBE ADD MUTEX, BUT ONLY ONE THREAD WRITES */
geometry_msgs::Twist currentVel;
sensor_msgs::LaserScan currentLidar;

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

void cmdVelCallback(const geometry_msgs::Twist &vel){
    currentVel = vel;
}

#ifndef TEST_EXEC
int main(int argc, char **argv){
    cout << "WEB 2DVIZ SIMULATION SERVER START" << endl;

    /* READ ARGS*/
    int tempargc = argc;
    char **tempargv = argv;
    (void) readArg(&tempargc, &tempargv); /* SKIP THE NAME ARG */
    args.port = (uint16_t) atoi(readArg(&tempargc, &tempargv));

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
            HttpHeader header = parseHttpReq(buffer.data(), size);

            int httpCode = HTTP_OK_CODE;
            string httpCodeStr = "OK";
            string response = "{}";
            /* ROUTE PACKET BY LOOKING AT VERB AND ROUTE */
            if(strcmp(header.route, GET_SYNC_ROUTE) == 0 && header.verb == parsing::HTTP_GET){
                /* REPLY 200 IS ENOUGH */
            }else if(strcmp(header.route, GET_CMD_VEL_ROUTE) == 0 && header.verb == parsing::HTTP_GET){
                response = "{\"linear\": [" + to_string(currentVel.linear.x) + ", " + to_string(currentVel.linear.y) + "], \"angular\": " + to_string(currentVel.angular.z) + "}";
            }else if(strcmp(header.route, POST_LIDAR_DATA_ROUTE) == 0 && header.verb == parsing::HTTP_POST){
                struct LidarData lastLidarData = parseLidarObj(&buffer.data()[header.bodyStartIndex], size - header.bodyStartIndex);
                currentLidar.angle_min = lastLidarData.minAngle;
                currentLidar.angle_max = lastLidarData.maxAngle;
                currentLidar.angle_increment = lastLidarData.angleStep;
                currentLidar.ranges = lastLidarData.distances;
                currentLidar.range_max = 12;
                currentLidar.range_min = 0;
            }else{
                cout << "ROUTE: " << header.route << " NOT KNOWN, 404" << endl;
                httpCode = HTTP_ERROR_CODE;
                httpCodeStr = "ERROR";
            }

            /* GENERATE RESPONSE */
            int responseSize = sprintf(buffer.data(), 
                HTTP_RESPONSE_FORMAT HTTP_HEADER_FIELD_DELIM
                "Content-Length: %lu" HTTP_HEADER_FIELD_DELIM
                "Access-Control-Allow-Origin: *" HTTP_HEADER_FIELD_DELIM
                HTTP_HEADER_FIELD_DELIM
                "%s", httpCode, httpCodeStr.c_str(), response.length(), response.c_str());
            int sent = 0;
            do{ sent += send(connectionFd, buffer.data(), responseSize, 0); }
            while(sent != responseSize);  
            close(connectionFd);
        }
    }};

    /* PUBLISH ROS TOPICS FOR CMD_VEL AND SENSORS */
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle SimServerNode;

    ros::Subscriber cmdVelListener = SimServerNode.subscribe(COMMAND_VEL_TOPIC, PUBLISHER_QUEUE_SIZE, cmdVelCallback);
    ros::Publisher lidarPublisher = SimServerNode.advertise<sensor_msgs::LaserScan>(LIDAR_DATA_TOPIC, PUBLISHER_QUEUE_SIZE);

    /* HANDLE ROS EVENTS ON MAIN THREAD */
    ros::Rate rate(PUBLISH_RATE);
    while(running && ros::ok()){
        /* PUBLISH LIDAR DATA */
        lidarPublisher.publish(currentLidar);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
#endif //TEST_EXEC