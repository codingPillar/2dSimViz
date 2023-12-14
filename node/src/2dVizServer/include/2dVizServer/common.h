#ifndef COMMON_H
#define COMMON_H

/* LIDAR DATA FORMAT
    {minAngle: number;
    maxAngle: number;
    angleStep: number;
    distances: number[];}
*/
#define LIDAR_DATA_MIN_ANGLE_KEY "minAngle"
#define LIDAR_DATA_MAX_ANGLE_KEY "maxAngle"
#define LIDAR_DATA_ANGLE_STEP_KEY "angleStep"
#define LIDAR_DATA_DISTANCES_KEY "distances"

#define ODOM_DATA_POS_X_KEY "x"
#define ODOM_DATA_POS_Y_KEY "y"
#define ODOM_DATA_POS_ANGLE_KEY "angle"

#define POST_LIDAR_DATA_ROUTE "/lidarData"
#define POST_ODOM_DATA_ROUTE "/odom"
#define GET_CMD_VEL_ROUTE "/cmdVel"
#define GET_SYNC_ROUTE "/sync"

#define EXIT_ERROR 1
#define SERVER_REQ_BUFFER 10
#define MAX_BUFFER_SIZE 2048

#define MAX_ROUTE_STR_SIZE 128
#define HTTP_HEADER_FIELD_DELIM "\r\n"
#define HTTP_END_HEADER_SEQ "\r\n\r\n"
#define HTTP_RESPONSE_FORMAT "HTTP/1.1 %u %s"
#define HTTP_OK_CODE 200
#define HTTP_ERROR_CODE 404

#endif //COMMON_H