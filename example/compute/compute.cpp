#include <iostream>

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define NODE_NAME "compute" 
#define COMPUTE_NODE_FREQ 10
#define QUEUE_SIZE 32
#define LIDAR_TOPIC "limo/scan"
#define CMD_VEL_TOPIC "cmd_vel"

#define FIELD_VIEW (3.1415F / 3.0F)
#define MAX_ALLOWED_DISTANCE 0.4F
#define DEFAULT_ANGULAR_VEL (0.50F)
#define DEFAULT_LINEAR_VEL 0.1F

using namespace std;

enum MOVING_STATE{
    MOVE_FORWARD,
    MOVE_ROTATING,
    MOVE_STOPPED
};

sensor_msgs::LaserScan lidarData;
bool lidarConsumed = true;

ros::Publisher *velPublisher = nullptr;

enum MOVING_STATE state = MOVE_FORWARD;

void lidarTopicCallback(const sensor_msgs::LaserScan &data){
    lidarData = data;
    lidarConsumed = false;
}

float getMinDistance(const sensor_msgs::LaserScan &lidarData, float fov){
    unsigned int length = fov * lidarData.ranges.size() / (lidarData.angle_max - lidarData.angle_min);
    if(length > lidarData.ranges.size()) length = lidarData.ranges.size();
    const unsigned int startIndex = (lidarData.ranges.size() - length) / 2;
    float minDistance = -1;
    for(unsigned int i = 0; i < length; i++)
        if(minDistance < 0 || minDistance > lidarData.ranges[startIndex + i]) minDistance = lidarData.ranges[startIndex + i];
    return minDistance;
}

void computeIteration(){
    if(lidarConsumed) return;
    switch(state){
        case MOVE_FORWARD:{
            /* CHECK MIN DISTANCE IN FRONT OF ROBOT */
            const float minDistance = getMinDistance(lidarData, FIELD_VIEW);
            geometry_msgs::Twist vel;
            /* OBSTACLE IN FRONT, CHANGE DIRECTION */
            if(minDistance < MAX_ALLOWED_DISTANCE){
                state = MOVE_ROTATING;
                cout << "CHANGING STATE TO ROTATING, MIN DISTANCE: " << minDistance << endl;
            } 
            else vel.linear.x = DEFAULT_LINEAR_VEL;
            velPublisher->publish(vel);
        }break;
        case MOVE_ROTATING:{
            geometry_msgs::Twist vel;
            vel.angular.z = DEFAULT_ANGULAR_VEL;
            velPublisher->publish(vel);
            /* TODO, CHECK WITH ODOM TO SEE IF TURNED ENOUGH */
        }break;
        case MOVE_STOPPED:{
            /* TODO, WE WANT TO SEND MESSAGE ONLY ONE TIME */
            geometry_msgs::Twist vel;
            velPublisher->publish(vel);
        }break;
    }
    //lidarConsumed = true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle computeNode;

    ros::Subscriber lidarSub = computeNode.subscribe(LIDAR_TOPIC, QUEUE_SIZE, lidarTopicCallback);
    ros::Publisher  velPublisher = computeNode.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, QUEUE_SIZE);
    ::velPublisher = &velPublisher;

    ros::Rate rate(COMPUTE_NODE_FREQ);
    while(ros::ok()){
        /* NODE LOGIC */
        computeIteration();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}