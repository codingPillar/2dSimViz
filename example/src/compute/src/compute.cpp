#include <cmath>
#include <cstdlib>
#include <iostream>

#include "ros/ros.h"
#include "tf/tf.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "nav_msgs/Odometry.h"
#include "ros/service_server.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"

#define NODE_NAME "compute" 

#define LIDAR_TOPIC "limo/scan"
#define CMD_VEL_TOPIC "cmd_vel"
#define ODOM_TOPIC "odom"

#define START_SRV "start"
#define STOP_SRV "stop"
#define RETURN_SRV "return"
#define CHANGE_ALGO_SRV "change"

#define MOVE_BASE_ACTION_SERVER_NAME "move_base"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define COMPUTE_NODE_FREQ 10
#define QUEUE_SIZE 32

#define FIELD_VIEW (3.1415F / 3.0F)
#define MAX_ALLOWED_DISTANCE 0.4F
#define DEFAULT_ANGULAR_VEL (0.50F)
#define DEFAULT_LINEAR_VEL 0.1F
#define ROTATION_GOAL_REACHED_UNVERTAINTY (0.15F)

using namespace std;

enum MOVING_STATE{
    MOVE_CHANGE_ALGO,
    MOVE_FORWARD,
    MOVE_ROTATING,
    MOVE_STOPPED,
    MOVE_RETURN,
};
enum EXPLORE_ALGORITHM{
    ALGO_RANDOM_WALK,
    ALGO_EXPLORE_LITE,
    NUM_SUPPORTED_ALGO
};

static sensor_msgs::LaserScan lidarData;
static bool lidarAvailable = false;

static geometry_msgs::Twist currentPosition;

static ros::Publisher *velPublisher = nullptr;
static MoveBaseClient *moveBaseClient = nullptr;

static enum MOVING_STATE state = MOVE_STOPPED;
static enum EXPLORE_ALGORITHM algorithm = ALGO_RANDOM_WALK;

static float previousRotation = 0;
static float rotationGoal = 0;
static bool handledStop = false;
static bool returning = false;
static ros::Time lastTime;

/* PRIVATE FUNCTIONS */
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
    /* POSITION ANGLE IS BETWEEN [-PI, PI], WE WANT IT IN RIGHT SPACE FROM [0, 2PI] */
    if(position.angular.z > 0) return position.angular.z;
    return 2 * M_PI + position.angular.z; 
}

static void stopRobot(){
    geometry_msgs::Twist vel;
    velPublisher->publish(vel);
    //moveBaseClient->cancelAllGoals();
    handledStop = true;
}

/* TOPIC CALLBACK FUNCTIONS */
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

/* SERVICE CALLBACK FUNCTIONS */
bool startSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    cout << "[COMPUTE] START SERVICE CALLED" << endl;
    state = MOVE_FORWARD;
    return true;
}

bool stopSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    cout << "[COMPUTE] STOP SERVICE CALLED" << endl;
    state = MOVE_STOPPED;
    return true;
}

bool returnSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    cout << "[COMPUTE] RETURN SERVICE CALLED" << endl;
    state = MOVE_RETURN;
    return true;
}

bool changeAlgoSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    cout << "[COMPUTE] CHANGE ALGORITHM SERVICE CALLED" << endl;
    /* TODO, IMPLEMENT */
    return true;
}

static void computeIteration(){
    if(!lidarAvailable) return;
    if(handledStop && state != MOVE_RETURN && state != MOVE_STOPPED){
        /* IN CASE RETURN BASE HAS BEEN INTERRUPTED */
        moveBaseClient->cancelAllGoals();
        handledStop = false;
    }
    switch(state){
        case MOVE_CHANGE_ALGO:{
            /* TODO, IMPLEMENT */
        }break;
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
            if(handledStop) break;
            stopRobot();
        }break;
        case MOVE_RETURN:{
            if(!handledStop){
                stopRobot();
                break;
            }
            if((ros::Time::now() - lastTime).sec >= 2){
                lastTime = ros::Time::now();
                cout << "CURRENT MOVE_BASE GOAL STATE: " << moveBaseClient->getState().toString() << endl;
            }
            if(moveBaseClient->getState().isDone()){
                cout << "MOVE_BASE GOAL ENDED WITH STATE: " << moveBaseClient->getState().toString() << endl;
                returning = false;
                state = MOVE_STOPPED;
                break;
            }
            if(returning) break;
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 0;
            goal.target_pose.pose.position.y = 0;
            goal.target_pose.pose.position.z = 0;
            goal.target_pose.pose.orientation.w = 1.0;
            moveBaseClient->sendGoal(goal);
            cout << "SENT MOVE_BASE GOAL" << endl;
            returning = true;
        };
    }
}

int main(int argc, char **argv){
    srand(time(0));
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle computeNode;

    /* TOPIC SETUP */
    ros::Subscriber lidarSub = computeNode.subscribe(LIDAR_TOPIC, QUEUE_SIZE, lidarTopicCallback);
    ros::Subscriber odomSub = computeNode.subscribe(ODOM_TOPIC, QUEUE_SIZE, odomDataCallback);
    ros::Publisher  velPublisher = computeNode.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, QUEUE_SIZE);
    ::velPublisher = &velPublisher;

    /* SERVICE SETUP */
    ros::ServiceServer startSrv = computeNode.advertiseService(START_SRV, startSrvCallback);
    ros::ServiceServer stopSrv  = computeNode.advertiseService(STOP_SRV, stopSrvCallback);
    ros::ServiceServer returnSrv  = computeNode.advertiseService(RETURN_SRV, returnSrvCallback);
    ros::ServiceServer changeSrv  = computeNode.advertiseService(CHANGE_ALGO_SRV, changeAlgoSrvCallback);

    /* ACTIONLIB SETUP */
    MoveBaseClient moveBaseClient(MOVE_BASE_ACTION_SERVER_NAME, true);
    ::moveBaseClient = &moveBaseClient;
    moveBaseClient.waitForServer();

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