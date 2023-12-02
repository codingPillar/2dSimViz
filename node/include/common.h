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

#define EXIT_ERROR 1
#define SERVER_REQ_BUFFER 10
#define MAX_BUFFER_SIZE 2048

#define HTTP_HEADER_FIELD_DELIM "\r\n"
#define HTTP_END_HEADER_SEQ "\r\n\r\n"
#define HTTP_RESPONSE_FORMAT "HTTP/1.1 %u %s"

#endif //COMMON_H