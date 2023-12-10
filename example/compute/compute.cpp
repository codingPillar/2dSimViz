#include <cmath>
#include <cstdlib>
#include <iostream>

#include "ros/ros.h"
#include <tf/tf.h>

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define NODE_NAME "compute" 
#define COMPUTE_NODE_FREQ 10
#define QUEUE_SIZE 32
#define LIDAR_TOPIC "limo/scan"
#define CMD_VEL_TOPIC "cmd_vel"
#define ODOM_TOPIC "odom"

#define FIELD_VIEW (3.1415F / 3.0F)
#define MAX_ALLOWED_DISTANCE 0.4F
#define DEFAULT_ANGULAR_VEL (0.50F)
#define DEFAULT_LINEAR_VEL 0.1F
#define ROTATION_GOAL_REACHED_UNVERTAINTY (0.15F)

using namespace std;

enum MOVING_STATE{
    MOVE_FORWARD,
    MOVE_ROTATING,
    MOVE_STOPPED
};

sensor_msgs::LaserScan lidarData;
bool lidarAvailable = false;

geometry_msgs::Twist currentPosition;

ros::Publisher *velPublisher = nullptr;

enum MOVING_STATE state = MOVE_FORWARD;
float previousRotation = 0;
float rotationGoal = 0;

static void lidarTopicCallback(const sensor_msgs::LaserScan &data){
    lidarData = data;
    lidarAvailable = true;
}

static void odomDataCallback(const nav_msgs::Odometry &data){
    tf::Quaternion quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                              data.pose.pose.orientation.w);
    tf::Matrix3x3 matrix(quaternion);
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    matrix.getRPY(roll, pitch, yaw);

    currentPosition.linear.x = data.pose.pose.position.x;
    currentPosition.linear.y = data.pose.pose.position.y;
    currentPosition.angular.z = yaw;
}

static float getMinDistance(const sensor_msgs::LaserScan &lidarData, float fov){
    unsigned int length = fov * lidarData.ranges.size() / (lidarData.angle_max - lidarData.angle_min);
    if(length > lidarData.ranges.size()) length = lidarData.ranges.size();
    const unsigned int startIndex = (lidarData.ranges.size() - length) / 2;
    float minDistance = -1;
    for(unsigned int i = 0; i < length; i++)
        if(minDistance < 0 || minDistance > lidarData.ranges[startIndex + i]) minDistance = lidarData.ranges[startIndex + i];
    return minDistance;
}

static float getOdomAngle(const geometry_msgs::Twist &position){
    /* POSITION ANGLE IS BETWEEN [-PI, PI], WE WANT IT IN RIGHT ORDER FROM [0, 2PI] */
    if(position.angular.z > 0) return position.angular.z;
    return 2 * M_PI + position.angular.z; 
}

static void computeIteration(){
    if(!lidarAvailable) return;
    switch(state){
        case MOVE_FORWARD:{
            /* CHECK MIN DISTANCE IN FRONT OF ROBOT */
            const float minDistance = getMinDistance(lidarData, FIELD_VIEW);
            geometry_msgs::Twist vel;
            /* OBSTACLE IN FRONT, CHANGE DIRECTION */
            if(minDistance < MAX_ALLOWED_DISTANCE){
                state = MOVE_ROTATING;
                rotationGoal = ((float)rand() / (float)RAND_MAX) * 2 * M_PI;
                previousRotation = getOdomAngle(currentPosition);
                cout << "CHANGING STATE TO ROTATING, MIN DISTANCE: " << minDistance << " DELTA ANGLE: " << rotationGoal << endl;
            } 
            else vel.linear.x = DEFAULT_LINEAR_VEL;
            velPublisher->publish(vel);
            lidarAvailable = false;
        }break;
        case MOVE_ROTATING:{
            geometry_msgs::Twist vel;
            vel.angular.z = DEFAULT_ANGULAR_VEL;
            velPublisher->publish(vel);
            const float angle = getOdomAngle(currentPosition);
            rotationGoal -= (abs(previousRotation - angle) < M_PI) ? abs(previousRotation - angle) : 2 * M_PI - abs(previousRotation - angle);
            previousRotation = angle;
            if(abs(rotationGoal) < ROTATION_GOAL_REACHED_UNVERTAINTY){
                rotationGoal = 0;
                state = MOVE_FORWARD;
            }
        }break;
        case MOVE_STOPPED:{
            /* TODO, WE WANT TO SEND MESSAGE ONLY ONE TIME */
            geometry_msgs::Twist vel;
            velPublisher->publish(vel);
        }break;
    }
}

int main(int argc, char **argv){
    srand(time(0));
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle computeNode;

    ros::Subscriber lidarSub = computeNode.subscribe(LIDAR_TOPIC, QUEUE_SIZE, lidarTopicCallback);
    ros::Subscriber odomSub = computeNode.subscribe(ODOM_TOPIC, QUEUE_SIZE, odomDataCallback);
    ros::Publisher  velPublisher = computeNode.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, QUEUE_SIZE);
    ::velPublisher = &velPublisher;

    geometry_msgs::Twist reset;
    velPublisher.publish(reset);

    ros::Rate rate(COMPUTE_NODE_FREQ);
    while(ros::ok()){
        /* NODE LOGIC */
        computeIteration();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}