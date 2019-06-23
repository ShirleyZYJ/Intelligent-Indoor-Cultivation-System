#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "apriltags_ros/AprilTagDetectionArray.h"


char byte[28] = {0};
char ID[11] = {0};
char id[3] ={0};
char Delta_x[7] = {0};
char Delta_y[7] = {0};
char Delta_yaw[6] = {0};
unsigned char flag;

float heading_direction;
int frame;
ssize_t size;

int seq;
