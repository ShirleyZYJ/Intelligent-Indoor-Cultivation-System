#include "ht_vision.h"
#include <string>
#include "math.h"
#include "dualmotor.h"
#include "singlemotor.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace std;

#define PI 3.1415926

bool init()
{
    //open USB serial port
    frame = open("/dev/ttyUSB2", O_RDWR | O_NOCTTY);
    
    if (frame < 0)
    {
       std::cout<< "Error " << "opening " << "/dev/ttyUSB2" << std::endl;
       exit(0);
       return false;
    }
    else
    {
       std::cout << "Open device succeeded" << std::endl;
    }
    //configure port and set the baud rate to 115200
    //please refer to the camera datasheet for more setting info
    struct termios options;
    tcgetattr(frame, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 28;

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    tcflush(frame,TCIFLUSH);
    tcsetattr(frame, TCSANOW, &options);
    
    return true;
}

int main(int argc, char** argv)
{
    init();
    ros::init(argc, argv, "ht_vision_data");
    ros::NodeHandle nh;
    //pubulish port data, echo the topic "/tag_detections" to see the result
    ros::Publisher ht_vision_data = nh.advertise<apriltags_ros::AprilTagDetectionArray>("/tag_detections", 1);
    std::cout << "Start capture data and publish" << std::endl;
    std::cout <<"-------------------------- Capture Started--------------------------------" <<std::endl;
    seq = 0;

    //start to read and decode data
    while(ros::ok())
    {
        apriltags_ros::AprilTagDetectionArray tag_detection_array;
        apriltags_ros::AprilTagDetection tag_detection;

        size = read(frame, &flag, 1);
        if (flag == 0x23)
        {
            size = read(frame, &byte, 28);
            if(byte[27] == 0x0D)
            {
                strncpy(ID, byte, 10);
                strncpy(id, ID + 8, 2);

                strncpy(Delta_x, byte + 10, 6);               

                strncpy(Delta_y, byte + 16, 6);
   
                strncpy(Delta_yaw, byte + 22, 5);

                apriltags_ros::AprilTagDetection tag_detection;
                //The accuracy of output pose data is 1000.00/1000000 = 0.01mm
                //The accuracy of output angle data is 100.00/10000 = 0.01 degree
                //stoi=>string to int
                tag_detection.pose.pose.position.x = std::stoi(Delta_y)/100000.0;
                tag_detection.pose.pose.position.y = std::stoi(Delta_x)/100000.0;
                tag_detection.pose.pose.position.z = std::stoi(Delta_yaw)/100.0;

 
                if (std::stoi(Delta_yaw)/100.0 > 180)
                //if angle < 180 => positive direction, otherwise negative direaction
                    heading_direction = (std::stoi(Delta_yaw)/100.0 - 360.0)/180.0 * PI;
                else
                    heading_direction = (std::stoi(Delta_yaw)/100.0)/180.0 * PI;
                //w=>pose on x-axis, z=>pose on y-axis
                tag_detection.pose.pose.orientation.w  = cos(heading_direction/2);
                tag_detection.pose.pose.orientation.z = sin(heading_direction/2);
                tag_detection.id = std::stoi(id);
                tag_detection.pose.header.stamp = ros::Time::now();

                seq++;
                tag_detection.pose.header.seq = seq;
                tag_detection_array.detections.push_back(tag_detection);
                ht_vision_data.publish(tag_detection_array);
            }
        }
        else
        {
                tag_detection.pose.header.stamp = ros::Time::now();

                seq++;
                tag_detection.pose.header.seq = seq;
                tag_detection_array.detections.push_back(tag_detection);
                ht_vision_data.publish(tag_detection_array);
        }        
    }     
}

