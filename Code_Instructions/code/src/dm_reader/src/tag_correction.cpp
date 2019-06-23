#include "tag_correction.h"
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define PI 3.1415926535898

tag_correction::tag_correction()
{
}

void
tag_correction::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(msg->poses.size())
    {   
         m_path = msg;
    }
    else
    {
         m_path = NULL;
    }
}

//passing const pointer into callback 
//for ApriltagDetectionArray, refer to ApriltagDetection.h
void 
tag_correction::absposeCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    if(msg->detections.size())
    {   
        abs_pose = boost::make_shared<const apriltags_ros::AprilTagDetection>(msg->detections[0]);
    }
    else
    {
        abs_pose = NULL;
    }
}

//initialise the ros publisher and subscriber
bool 
tag_correction::init(ros::NodeHandle& nh)
{
    //set up path subscriber, in callback function, subscribe the path topic which is a series of Poses    
    m_pathSub = nh.subscribe<nav_msgs::Path>("path", 1, boost::bind(&tag_correction::pathCallback, this, _1));
    //set up pose estimated by vision algorithm which should be periodically updated when tag is detected
    m_absposeSub  = nh.subscribe<apriltags_ros::AprilTagDetectionArray>("/tag_detections", 1, boost::bind(&tag_correction::absposeCallback, this, _1));
    //initialize current position
    m_cur << 0,
             0,
             0;

    m_goal << 1,
              2,
              3;
    
    if( m_cur[0] != 0 || m_cur[1] != 0 || m_cur[2] != 0){
        printf("Please adjust the robot to its starting position!\n");
    }
    //initialise state
    m_move = IDLE; 
    //initialize two serial port for walking motor and lifting motor
    char *dev_1;
    char *dev_2;
    dev_1 = "/dev/ttyUSB0";
    dev_2 = "/dev/ttyUSB1";

//locomotor initialization
    locomotor = boost::make_shared<dualmotor>(0x01, 0x02, dev_1);
    locomotor->power_on(); 
    locomotor->init();
    vel_loco = locomotor->get_velocity();
//liftmotor initialization
    liftmotor = boost::make_shared<singlemotor>(0x01, dev_2);
    //reset position to the height of 5th shelf
    liftmotor->reset_position();
    liftmotor->power_on();
    //control mode is set to position control
    liftmotor->init(0);
    return true;
}

void
tag_correction::running(void)
{
    cycle_start = ros::Time::now();

    //adjust angle according to dm code
    int count_detect = 0;
    while(ros::ok())
    {   //tag is detected
        if(abs_pose)
        {
            float delta_yaw;
            delta_yaw = abs_pose->pose.pose.position.z;

            if(abs(360-delta_yaw)>5){
                if(delta_yaw<=180){
                    locomotor->set_velocity(50,50);
                    if(abs(delta_yaw)<10)
                    locomotor->set_velocity(0,0);
                    printf("Yaw %f corrected!\n", delta_yaw);
                    break;
                }
                else
                {   delta_yaw=360-delta_yaw;
                    locomotor->set_velocity(-50,-50);
                    if(abs(delta_yaw)<3)
                    locomotor->set_velocity(0,0);
                    printf("Yaw %f corrected!\n", delta_yaw);
                    break;
                }
            }
        }
        //tag is not detected
        else
        {
            locomotor->set_velocity(0,0);
            usleep(10000);
            count_detect++;
        }
        ros::spinOnce();
        if (count_detect>1000)
        {
            ROS_WARN("No Tag detected when stoped");
            ros::shutdown();
            exit(1);
        }

        // locomotor->set_velocity(-200,200);
        // int delta_x=m_goal[0]-m_cur[0];
        // usleep(8333333*delta_x);

        // locomotor->set_velocity(200,200);
        // int delta_angle = abs(m_goal[1]-m_cur[1]); 
        // usleep(3403391*delta_angle);   

        // int delta_level;
        // delta_level = m_goal[2] -m_cur[2];
        // liftmotor->set_position(delta_level);
        // sleep(20);

    }

}



// //switch status according to the command
//     switch (m_move)
//     {
//         case IDLE:
//         {   
//             locomotor->set_velocity(0,0);
//             int id = abs_pose->id;

//             //adjust angle according to dm code
//             int count_detect = 0;
//             while(ros::ok())
//             {   //tag is detected
//                 if(abs_pose)
//                 {
//                     float delta_yaw;
//                     delta_yaw = abs_pose->pose.pose.position.z;
 
//                     if(abs(360-delta_yaw)>5){
//                         if(delta_yaw<=180){
//                             locomotor->set_velocity(50,50);
//                             if(abs(delta_yaw)<3)
//                             locomotor->set_velocity(0,0);
//                             printf("Yaw %f corrected!\n", delta_yaw);
//                             break;
//                         }
//                         else
//                         {   delta_yaw=360-delta_yaw;
//                             locomotor->set_velocity(-50,-50);
//                             if(abs(delta_yaw)<3)
//                             locomotor->set_velocity(0,0);
//                             printf("Yaw %f corrected!\n", delta_yaw);
//                             break;
//                         }
//                     }
//                 }
//                 //tag is not detected
//                 else
//                 {
//                     locomotor->set_velocity(0,0);
//                     usleep(10000);
//                     count_detect++;
//                 }
//                 ros::spinOnce();
//                 if (count_detect>1000)
//                 {
//                     ROS_WARN("No Tag detected when stoped");
//                     ros::shutdown();
//                     exit(1);
//                 }

//                 //forward
//                 if(m_goal[0]-m_cur[0]>0){
//                     m_move = FORWARD;
//                 }
//                 //backward
//                 if(m_goal[0]-m_cur[0]<0){
//                     m_move = BACKWARD;
//                 }
//                 //turn
//                 if(m_goal[1]!=m_cur[1]){
//                     m_move = TURN;
//                 }
//                 //lift
//                 if(m_goal[2]!=m_cur[2]){
//                     m_move = LIFT;
//                 }

//             }
//         }
//         //move forward 
//         case FORWARD:
//         {
//             locomotor->set_velocity(200,-200);
//             usleep(8333333);
//             if(abs_pose){
//                 m_goal[0]=m_goal[0]-1;
//                 m_move = IDLE;
//             }
//             break;
//         }

//         case BACKWARD:
//         {
//             locomotor->set_velocity(-200,200);
//             usleep(8333333);
//             if(abs_pose){
//                 m_goal[0]=m_goal[0]+1;
//                 m_move = IDLE;
//             }
//             break;
//         }

//         case LIFT:
//         {
//             locomotor->set_velocity(0,0);
//             int delta_level;
//             delta_level = m_goal[2] -m_cur[2];
//             liftmotor->set_position(delta_level);
//             sleep(20);
//             m_cur[2] = m_goal[2];
//             m_move = IDLE;   
//             break;                        
//         }

//         case TURN:
//         {   
//             int delta_angle;
//             if(m_goal[1]-m_cur[1]<0 || abs(m_goal[1]-m_cur[1]) >2){
//                 locomotor->set_velocity(200,200);
//                 //forward velocity is 0.12m/s for (200,-200)
//                 //PI*B/(4*0.12) = 3.403391;
//                 //B is distance between two driving wheel, in our case 0.52m
//                 //3.4 is the time for robot at 0.12m/s to turn an angle of 90
//                 delta_angle = abs(m_goal[1]-m_cur[1]); 
//                 usleep(3403391*delta_angle);
//                 m_cur[1]=m_goal[1];
//                 m_move = IDLE;
//                 break;
//             }
//             if(m_goal[1]-m_cur[1]<0 || abs(m_goal[1]-m_cur[1]) <=2){
//                 locomotor->set_velocity(-200,-200);
//                 delta_angle = abs(m_goal[1]-m_cur[1]); 
//                 usleep(3403391*delta_angle);
//                 m_cur[1]=m_goal[1];
//                 break;
//             }
//             if(m_goal[1]-m_cur[1]>0 || abs(m_goal[1]-m_cur[1]) <=2){
//                 locomotor->set_velocity(200,200);
//                 delta_angle = abs(m_goal[1]-m_cur[1]); 
//                 usleep(3403391*delta_angle);
//                 m_cur[1]=m_goal[1];
//                 m_move = IDLE;
//                 break;
//             }
//             if(m_goal[1]-m_cur[1]>0 || abs(m_goal[1]-m_cur[1]) >2){
//                 locomotor->set_velocity(-200,-200);
//                 delta_angle = abs(m_goal[1]-m_cur[1]); 
//                 usleep(3403391*delta_angle);
//                 m_cur[1]=m_goal[1];
//                 m_move = IDLE;
//                 break;
//             }

//         }
//     }


void 
tag_correction::shutdown()
{
    locomotor->shut_down();
    liftmotor->shut_down();
    printf("Shut down motor\n");
}
