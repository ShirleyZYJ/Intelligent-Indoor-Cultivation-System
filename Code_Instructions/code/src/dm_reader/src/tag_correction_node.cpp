#include "tag_correction.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_correction");
    ros::NodeHandle nh;
    tag_correction corrector;

    if (ros::ok())
    {
       ros::spinOnce();
    }
    
    while(!corrector.init(nh))
    {
       ROS_INFO("Wait for correction");       
       ros::spinOnce();
       sleep(1);
    }

    while(nh.ok())
    {
       std::cout <<"-------------Driver Initialization Finished--------------" <<std::endl;
       corrector.running();
       ros::spinOnce();
    }

    corrector.shutdown();
}
