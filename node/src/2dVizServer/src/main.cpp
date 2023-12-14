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
#include "ros/time.h"

#define JSON_PARSE_IMPLEMENTATION
#include "jsonParse.hpp"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define NODE_NAME "SimViz2dServer"
#define COMMAND_VEL_TOPIC "cmd_vel"
#define LIDAR_DATA_TOPIC "limo/scan"
#define ODOM_DATA_TOPIC "odom"

#define LASER_LINK_NAME "laser_link"

#define PUBLISHER_QUEUE_SIZE 32
#define PUBLISH_RATE 20

using namespace std;
using namespace parsing;

struct Args{
    uint16_t port;
    std::string baseLink;
} static args; 

static bool running = true;
/* MAYBE ADD MUTEX, BUT ONLY ONE THREAD WRITES */
static geometry_msgs::Twist currentVel;
static sensor_msgs::LaserScan currentLidar;
static nav_msgs::Odometry currentOdom;

/* TF base_link -> odom */
static tf::TransformBroadcaster *tfBraudcast;

static char *readArg(int *argc, char ***argv){
    if(*argc == 0){
        cout << "USAGE\nnode <port>" << endl;
        exit(EXIT_ERROR);
    }
    char *temp = (*argv)[0];
    (*argv)++;
    (*argc)--;
    return temp;
}

/* @odom: LAST POSITION RECORDED BY ODOMETER */
void baseLinkToOdomBraudcast(const nav_msgs::Odometry &odom){
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0));
    transform.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
    tfBraudcast->sendTransform(tf::StampedTransform(transform, ros::Time::now(), ODOM_DATA_TOPIC, args.baseLink));
}

static void cmdVelCallback(const geometry_msgs::Twist &vel){
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
    /* TODO, READ FROM COMMAND LINE BASE LINK NAME */
    args.baseLink = "base_link";

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
            /* TODO, MAYBE MANIPULATE VAR INSTEAD OF BREAK */
            bool connected = true;
            while(connected){
                /* TODO, ADD DELAY FOR WHEN NOT RECEIVING A MESSAGE WITH THAT TIME, BREAK AND CLOSE CONNECTION */
                int size = recv(connectionFd, buffer.data(), buffer.size(), 0);
                if(size <= 0){
                    cout << "CLIENT DISCONNECTED, NEXT CONNECTION" << endl;
                    break;
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
                    struct jsonParse::JsonObj json = jsonParse::parseJson(&buffer.data()[header.bodyStartIndex], size - header.bodyStartIndex);
                    struct LidarData lastLidarData = parseLidarObj(json);
                    //cout << &buffer.data()[header.bodyStartIndex] << endl;
                    currentLidar.header.stamp = ros::Time::now();
                    currentLidar.header.frame_id = LASER_LINK_NAME;
                    currentLidar.angle_min = lastLidarData.minAngle;
                    currentLidar.angle_max = lastLidarData.maxAngle;
                    currentLidar.angle_increment = lastLidarData.angleStep;
                    currentLidar.ranges = lastLidarData.distances;
                    currentLidar.range_max = 12;
                    currentLidar.range_min = 0;
                }else if(strcmp(header.route, POST_ODOM_DATA_ROUTE) == 0){
                    struct jsonParse::JsonObj json = jsonParse::parseJson(&buffer.data()[header.bodyStartIndex], size - header.bodyStartIndex);
                    struct OdomData odom = parseOdomObj(json);
                    tf2::Quaternion myQuaternion;
                    myQuaternion.setRPY(0, 0, odom.angle);
                    myQuaternion.normalize();
                    currentOdom.child_frame_id = ODOM_DATA_TOPIC;
                    currentOdom.pose.pose.position.x = odom.x;
                    currentOdom.pose.pose.position.y = odom.y;

                    currentOdom.pose.pose.orientation.x = myQuaternion.x();
                    currentOdom.pose.pose.orientation.y = myQuaternion.y();
                    currentOdom.pose.pose.orientation.z = myQuaternion.z();
                    currentOdom.pose.pose.orientation.w = myQuaternion.w();
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
                int status = send(connectionFd, buffer.data(), responseSize, 0);
                if(status <= 0) break;
            }
            close(connectionFd);
        }
    }};

    /* PUBLISH ROS TOPICS FOR CMD_VEL AND SENSORS */
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle SimServerNode;

    tf::TransformBroadcaster tfBraudcast;
    ::tfBraudcast = &tfBraudcast;

    ros::Subscriber cmdVelListener = SimServerNode.subscribe(COMMAND_VEL_TOPIC, PUBLISHER_QUEUE_SIZE, cmdVelCallback);
    ros::Publisher lidarPublisher = SimServerNode.advertise<sensor_msgs::LaserScan>(LIDAR_DATA_TOPIC, PUBLISHER_QUEUE_SIZE);
    ros::Publisher odomPublisher = SimServerNode.advertise<nav_msgs::Odometry>(ODOM_DATA_TOPIC, PUBLISHER_QUEUE_SIZE);

    currentOdom.child_frame_id = ODOM_DATA_TOPIC;
    currentOdom.header.stamp = ros::Time::now();
    currentOdom.header.frame_id = ODOM_DATA_TOPIC;
    currentOdom.pose.pose.orientation.w = 1;

    /* HANDLE ROS EVENTS ON MAIN THREAD */
    ros::Rate rate(PUBLISH_RATE);
    while(running && ros::ok()){
        /* PUBLISH LIDAR DATA */
        lidarPublisher.publish(currentLidar);
        odomPublisher.publish(currentOdom);
        baseLinkToOdomBraudcast(currentOdom);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
#endif //TEST_EXEC