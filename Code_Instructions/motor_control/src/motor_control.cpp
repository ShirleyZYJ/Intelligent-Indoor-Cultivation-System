#include "motor_control.h"
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


motion_controller::motion_controller()
{
}

bool 
motion_controller::indicator(double theta_x, double theta_y, double threshold)
{
    return (2*PI-fabs(theta_x-theta_y) < threshold)||(fabs(theta_x - theta_y) < threshold);
}

double 
motion_controller::angle(double goal, double start)
{
    if(fabs(goal-start) <= PI)
       return goal-start;
    else if ((goal-start)>PI)
       return goal-start - 2*PI;
    else
       return goal-start + 2*PI;
}

Eigen::Matrix3f
motion_controller::qToRotation(Eigen::Quaternion<float> q)
{
    Eigen::Matrix3f temp;

    temp << 1-2*(q.y()*q.y()+q.z()*q.z()), 2*(q.x()*q.y()-q.w()*q.z())  , 2*(q.w()*q.y()+q.x()*q.z())  ,
            2*(q.x()*q.y()+q.w()*q.z())  , 1-2*(q.x()*q.x()+q.z()*q.z()), 2*(q.y()*q.z()-q.w()*q.x())  ,
            2*(q.x()*q.z()-q.w()*q.y())  , 2*(q.y()*q.z()+q.w()*q.x())  , 1-2*(q.x()*q.x()+q.y()*q.y());
    return temp;
}

double
motion_controller::getYawFromMat(Eigen::Matrix3f mat)
{
    return atan2(mat(1,0), mat(0,0));
}

double 
motion_controller::mod2pi(double angle)
{
    if(fabs(angle) <= PI)
         return angle;
    else if (angle < -PI)
         return 2*PI+angle;
    else
         return angle-2*PI;
}

void
motion_controller::pathCallback(const nav_msgs::Path::ConstPtr& msg)
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
motion_controller::absposeCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
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

void 
motion_controller::absposeCallback1(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    if(msg->detections.size())
    {   
        abs_pose1 = boost::make_shared<const apriltags_ros::AprilTagDetection>(msg->detections[0]);
    }
    else
    {
        abs_pose1 = NULL;
    }
}

//initialise the ros publisher and subscriber
bool 
motion_controller::init(ros::NodeHandle& nh)
{
    //set up path subscriber, in callback function, subscribe the path topic which is a series of Poses    
    m_pathSub = nh.subscribe<nav_msgs::Path>("path", 1, boost::bind(&motion_controller::pathCallback, this, _1));
    //set up pose estimated by vision algorithm which should be periodically updated when tag is detected
    m_absposeSub  = nh.subscribe<apriltags_ros::AprilTagDetectionArray>("tag_detections", 1, boost::bind(&motion_controller::absposeCallback, this, _1));
    m_absposeSub1 = nh.subscribe<apriltags_ros::AprilTagDetectionArray>("tag_detections1", 1, boost::bind(&motion_controller::absposeCallback1, this, _1));
    //set up pose advertiser, publish the estimated pose, the estimation is done by odometry and vision algorithm
    m_posePub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    //initialize two serial port for walking motor and lifting motor
    char *dev_1;
    char *dev_2;
    dev_1 = "/dev/ttyUSB0";
    dev_2 = "/dev/ttyUSB1";

//  TODO: Add localisation initialisation process or manually tune it. If robot is not at the pose, turn on tag detector first
// initialize start pose, first element is x displayment, second element is y displayment, and third one is current heading direction in rad
    m_state << 0,
               2.88,
               0;

    sleep(0.2);

    //initialise state and parameter
    m_move = IDLE;
    //initialize waypoint reach status
    block_path_receiver = false;
    count =0;
    //set the tag resolution
    m_resolution = 0.96;

//locomotor initialization
    locomotor = boost::make_shared<dualmotor>(0x01, 0x02, dev_1);
    locomotor->init();
    locomotor->power_on(); 
    vel_loco = locomotor->get_velocity();
    sleep(0.5);
//liftmotor initialization
    liftmotor = boost::make_shared<singlemotor>(0x01, dev_2);
    //reset position to the height of 5th shelf
    liftmotor->reset_position();
    liftmotor->power_on();
    //control mode is set to position control
    liftmotor->init(0);

    return true;
}

double 
motion_controller::distance2d(Eigen::Vector3d point_end, Eigen::Vector3d point_start)
{
    return sqrt((point_end[0]-point_start[0])*(point_end[0]-point_start[0]) + (point_end[1]-point_start[1])*(point_end[1]-point_start[1]));
}

void
motion_controller::running(void)
{
    cycle_start = ros::Time::now();

//receive path and when received block path receiver until mission completed
    if (m_path && !block_path_receiver)
    {
        nav_msgs::Path temp_path = *m_path;
        std::vector<geometry_msgs::PoseStamped> temp_path_vector;
        extracted_path.clear();
        for( std::vector<geometry_msgs::PoseStamped>::iterator itr = temp_path.poses.begin() ; itr != temp_path.poses.end(); ++itr)
        {
            temp_path_vector.push_back(*itr);
        }

        if(!temp_path_vector.empty())
        {
             ROS_INFO("Path Received!");
        }
        p_count++;

        while(!temp_path_vector.empty())
        {
            extracted_path.push_back(temp_path_vector.back());
            temp_path_vector.pop_back();
        }
        block_path_receiver = true;
    }

//switch status according to the command
    switch (m_move)
    {
        //move forward 
        case FORWARD:
        {
            //compute linear vel of the robot
            v =  sqrt(dist)*v_max/(1 + gamma*k*k)*10000/PI;
            //compute angular vel of the robot w=(vr-vl)/B
            //vr and vl are velocity of right and left wheel, B refers to baseline length.
            //0.385=0.77/2
            gain = 0.385*v*k;
            //retricted the angular to be less than a set value, in this case 600
            if (fabs(gain) > 600)
            {
                gain = boost::math::sign(gain) * 600;
            }
            //restricted the linear vel to be less than a set value, in this case 1000
            if(fabs(v)>1000)
            {
                v = boost::math::sign(v)*1000;
            }
            //set motor rotation velocity
            //floor() => round number
            locomotor->set_velocity(floor(v)+floor(gain),-floor(v)+floor(gain));

            //when the euler distance is less than 100mm, achieved waypoint
            //jump out forward status if the euler distance between current position and waypoint position is less than 30mm
            if (distance2d(m_state, m_cmd) < 0.09 && indicator(m_state[2],m_cmd[2],0.5))
            {
                m_move = IDLE;
            }
            break;
        }

        case BACKWARD:
        {
            v =  sqrt(dist)*v_max/(1 + gamma*k_back*k_back) *10000/PI;

            gain = 0.385*v*k_back;

            if (fabs(gain) > 600)
            {
                gain = boost::math::sign(gain) * 600;
            }

            if(fabs(v)>1000)
            {
                v = boost::math::sign(v)*1000;
            }
            
            locomotor->set_velocity(-floor(v)+floor(gain),floor(v)+floor(gain));
            if (distance2d(m_state, m_cmd) < 0.09 && indicator(m_state[2],m_cmd[2],0.5))
            {
                m_move = IDLE;
            }
            break;
        }
        
        case LIFT:
        {
            // TODO: Lift height should be determined
            locomotor->set_velocity(0,0);
            liftmotor->power_on();
            liftmotor->set_position(0.45);
            sleep(10)
            printf("Start Test\n");

            // pose correction code inserted here  first make sure tag is attached vertically, second camera has no pitch angle relative to the vehicle
            if ((fork_count%2) == 1)
            {
                printf("Start Correction!\n");
                int count_detect = 0;
                while(ros::ok())

                {
                    if(abs_pose1)
                    {
                        Eigen::Quaternion<float> quat;
                        quat.w() = abs_pose1->pose.pose.orientation.w;
                        quat.x() = abs_pose1->pose.pose.orientation.x;
                        quat.y() = abs_pose1->pose.pose.orientation.y;
                        quat.z() = abs_pose1->pose.pose.orientation.z;

                        Eigen::Matrix3f Rotation;

                        Rotation = qToRotation(quat);

                        Eigen::Matrix3f FixTF;
                        FixTF << 1, 0,  0,
                                 0, 0, -1,
                                 0, 1,  0;

                        Rotation = FixTF * Rotation.inverse();

                        float yaw_error;
                    
                        yaw_error = getYawFromMat(Rotation);
                    
                        gain = -3850*(k_angle*yaw_error)/3.1415926;
                        if(fabs(gain)>150)
                        {
                            gain = boost::math::sign(gain) * 150;
                        }
                        locomotor->set_velocity(floor(gain), floor(gain));
                        if (fabs(yaw_error*180/3.1415926) < 0.5)
                        {
                            locomotor->set_velocity(0,0);
                            printf("Yaw %f corrected!\n", yaw_error*180/3.1415926);
                            break;
                        }
                    }
                    else
                    {
                        locomotor->set_velocity(0, 0);
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
                }
 
                count_detect = 0;
                while(ros::ok())
                {
                    if(abs_pose1)
                    {

                        Eigen::Quaternion<float> quat;
                        quat.w() = abs_pose1->pose.pose.orientation.w;
                        quat.x() = abs_pose1->pose.pose.orientation.x;
                        quat.y() = abs_pose1->pose.pose.orientation.y;
                        quat.z() = abs_pose1->pose.pose.orientation.z;

                        Eigen::Matrix3f Rotation;
                        Eigen::Vector3f translation;
                        translation << abs_pose1->pose.pose.position.x,
                                       abs_pose1->pose.pose.position.y,
                                       abs_pose1->pose.pose.position.z;

                        Rotation = qToRotation(quat);

                        translation = translation;

                        float x_error;
                    
                        x_error = translation[0];
                    
                        v = 0.25*(x_error-0.003)*10000/3.1415926;
//                        printf("x is %f\n", x_error);
//                        printf("v is %f\n\n", floor(v));
                        if(fabs(v)>150)
                        {
                            v = boost::math::sign(v) * 150;
                        }
                        locomotor->set_velocity(floor(v), -floor(v));
                        if (fabs(x_error-0.003) < 0.003)
                        {
                            locomotor->set_velocity(0, 0);
                            printf("x %f corrected!\n", (x_error-0.006));
                            break;
                        }
                    }
                    else
                    {
                        locomotor->set_vel(0, 0);
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
                }
            }

            if ((fork_count%2) == 0)
            {
                int count_detect = 0;

                while(ros::ok())
                {
                    if (abs_pose)
                    {
                        Eigen::Quaternion<float> quat;
                        quat.w() = abs_pose->pose.pose.orientation.w;
                        quat.x() = abs_pose->pose.pose.orientation.x;
                        quat.y() = abs_pose->pose.pose.orientation.y;
                        quat.z() = abs_pose->pose.pose.orientation.z;

                        Eigen::Matrix3f Rotation;

                        Rotation = qToRotation(quat);

                        Eigen::Matrix3f fixTF;
                        fixTF << 1, 0,  0,
                                 0, -1, 0,
                                 0, 0, -1;

                        Rotation = Rotation.inverse()*fixTF;

                        m_state[2] = getYawFromMat(Rotation);
                        gain = 3850*(k_angle*angle(m_cmd[2],m_state[2]))/PI;
                        if (fabs(gain) > 100)
                        {
                            gain = boost::math::sign(gain) * 100;
                        }
                        
                        if (fabs(gain) >100)
                        {
                            ros::shutdown();
                            exit(1);
                        }

                        locomotor->set_velocity(floor(gain),floor(gain));
                        if (indicator(m_cmd[2], m_state[2], 0.008))
                        {
                            locomotor->set_velocity(0, 0);
                            printf("Corrected!\n");
                            break;
                        }
                    }
                    else
                    {
                        locomotor->set_velocity(0, 0);
                        usleep(10000);
                        count_detect++;
                    }
                    ros::spinOnce();

                    if (count_detect>1000)
                    {
                        locomotor->set_velocity(0, 0);
                        ROS_WARN("No Tag detected when stoped");
                        //ros::shutdown();
                        //exit(1);
                    }
                }
            }

            // if we carry out hand hold barcode test, after correction, stop
//               ros::shutdown();
//               exit(1);

            locomotor->shut_down();
            sleep(0.5);

            if ((fork_count%2) == 1)
                 liftmotor->set_position(-0.5);
            else
                 liftmotor->set_position(-0.45);
            sleep(3);

            liftmotor->shut_down();
            locomotor->power_on();
            sleep(0.5);
            m_move = IDLE;

            break;
        }

        case TURN:
        {   
            //compute the linear vel of the robot
            v = cos(alpha)*dist*k_dist *10000/PI;
            //rectricrted the liear speed to be less than a set value
            if(fabs(v)>300)
            {
                v = boost::math::sign(v)*300;
            }

            gain = 3850*(k_angle*angle(m_cmd[2],m_state[2]))/PI;
            if (fabs(gain) > 400)
            {
                gain = boost::math::sign(gain) * 400;
            }
            locomotor->set_velocity(floor(v)+floor(gain),-floor(v)+floor(gain));

            if (indicator(m_state[2],m_cmd[2],0.01))
            {
                m_move = IDLE;
            }
            break;
        }
  
        case IDLE:
        {
           locomotor->set_velocity(0,0);
           /*check current path to follow is empty or not
             if it is empty, publish block_path_receive status to be false and allow robot to receive new path 
             if it is not, execute next commanded waypoint in the path to follow */
           if (extracted_path.size()!=0)
           {
               geometry_msgs::PoseStamped temp_pose = extracted_path.back();
               float yaw_ = 2*atan2(temp_pose.pose.orientation.z,temp_pose.pose.orientation.w);
               m_cmd << temp_pose.pose.position.x * m_resolution,
                        temp_pose.pose.position.y * m_resolution,
                        angle(yaw_,0);

               printf("Next Commanded Pose is (%f, %f, %f)\n", m_cmd[0], m_cmd[1], m_cmd[2]);
               /* check next command waypoint is in the front of the robot or behind the robot, 
                  or the robot need to pure rotate to reach the next commanded waypoint, or lift*/
               if ( (fabs(m_cmd[0] - m_state[0])>0.5) && (fabs(m_cmd[1] - m_state[1])>0.5) )
               {
                   printf("Invalid commanded position received!\n");
                   locomotor->shut_down();
                   ros::shutdown();
                   exit(1);
               }
               if ( (fabs(m_cmd[0] - m_state[0])>0.5) || (fabs(m_cmd[1] - m_state[1])>0.5) )
               {
                   if (fabs(angle(m_cmd[2],m_state[2]))>0.5)
                   {
                       printf("Invalid commanded pose orientation received!\n");
                       locomotor->shut_down();
                       ros::shutdown();
                       exit(1);
                   }
                   if (fabs(m_cmd[0] - m_state[0])>0.5)
                   {
                       if (cos(m_state[2]) *  (m_cmd[0] - m_state[0]) > 0)
                           m_move = FORWARD;
                       else
                           m_move = BACKWARD;
                   }
                   else
                   {
                       if (sin(m_state[2]) *  (m_cmd[1] - m_state[1]) > 0)
                           m_move = FORWARD;
                       else
                           m_move = BACKWARD;
                   }
                   if (m_move == FORWARD)
                       printf("Move Forward!\n");
                   else
                       printf("Move Backward!\n");
               }

               else if (fabs(m_cmd[2] - m_state[2])>0.5)
               {
                   m_move = TURN;
                   printf("Turn Around!\n");
               }
               
               else if (temp_pose.pose.position.z!=0)
               {
                   m_move = LIFT;
                   fork_count++;
               }

               else
                    m_move = IDLE;
               extracted_path.pop_back();
           }
           else
           {   //wait for new path and publish current pose
               block_path_receiver = false;
               geometry_msgs::PoseStamped pose_msg;
         
               pose_msg.header.frame_id = "world";
               pose_msg.header.stamp = ros::Time::now();
               pose_msg.pose.position.x = m_state[0];
               pose_msg.pose.position.y = m_state[1];
               if (block_path_receiver)
                  pose_msg.pose.position.z = 1;
               else
                  pose_msg.pose.position.z = 0;

               pose_msg.pose.orientation.w = cos(m_state[2]/2);
               pose_msg.pose.orientation.x = 0;
               pose_msg.pose.orientation.y = 0;
               pose_msg.pose.orientation.z = sin(m_state[2]/2);
               m_posePub.publish(pose_msg);
               printf("IDLE!\n");
               sleep(1);
               m_move = IDLE;
           }

           break;
        }
    }
    //set the final vr and vl and publish it, 
    vel_loco.first = (vel_loco.first + locomotor->get_velocity().first)/2;
    vel_loco.second = (vel_loco.second + locomotor->get_velocity().second)/2;

    //delta t
    cycle_period = (ros::Time::now() - cycle_start).toSec();

    //odometry update (read encoder)
    //x = x + v*cos(theta t)*(delta t)
    m_state[0] = m_state[0] + (-vel_loco.second+vel_loco.first)/2 * cos(m_state[2]) * 0.018849555 * cycle_period/60;
    //y = y + v*sin(theta t)*(delta t)
    m_state[1] = m_state[1] + (-vel_loco.second+vel_loco.first)/2 * sin(m_state[2]) * 0.018849555 * cycle_period/60;        
    //theta t = theta t + (v/r) * (delta t)
    //r is the length of the robot /2 = 0.77/2
    // 1/5.30516503 = 0.01884955
    m_state[2] = m_state[2] + (vel_loco.first + vel_loco.second)/0.77 * 0.018849555 * cycle_period/60;


    if (abs_pose)

    {
        //if tag is detected, udpate absolute current pose
        int id;
        id = abs_pose->id - 3;

        Eigen::Quaternion<float> quat;
        quat.w() = abs_pose->pose.pose.orientation.w;
        quat.x() = abs_pose->pose.pose.orientation.x;
        quat.y() = abs_pose->pose.pose.orientation.y;
        quat.z() = abs_pose->pose.pose.orientation.z;

        Eigen::Matrix3f Rotation;
        Eigen::Vector3f translation;
        translation << abs_pose->pose.pose.position.x,
                       abs_pose->pose.pose.position.y,
                       abs_pose->pose.pose.position.z;

        Rotation = qToRotation(quat);

        translation = -Rotation.inverse()*translation;

        //transform matrix between camera frame and the robot base frame
        /* The mobile robot aligned with a global axis.
           cosθ   sinθ    0
           –sinθ  cosθ    0
           0      0       1
           In our case θ = 90 degree, thus,
           0   1  0
           -1  0  0
           0   0  1
        */
        Eigen::Matrix3f fixTF;
        fixTF << 1, 0,  0,
                 0, -1, 0,
                 0, 0, -1;

        Rotation = Rotation.inverse()*fixTF;

        m_state[0] = translation[0] + m_resolution * (id%10);
        m_state[1] = translation[1] + m_resolution * floor(id/10.0);
        m_state[2] = getYawFromMat(Rotation) + 0.04;
    }
    //published current estimated pose by the robot
    geometry_msgs::PoseStamped pose_msg;
         
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = m_state[0];
    pose_msg.pose.position.y = m_state[1];
    if (block_path_receiver)
        pose_msg.pose.position.z = 1;
    else
        pose_msg.pose.position.z = 0;

    pose_msg.pose.orientation.w = cos(m_state[2]/2);
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = sin(m_state[2]/2);

    m_posePub.publish(pose_msg);

    //prepare for the feedback data for differential control
    //change of linear speed in direction of Y
    delta_y = m_cmd[1]-m_state[1];
    if(fabs(m_cmd[1]-m_state[1]) > 1.6)
    {
        delta_y = boost::math::sign(m_cmd[1]-m_state[1]) * 1.6;
    }
    //change of linear speed in direction of X
    delta_x = m_cmd[0]-m_state[0];
    if(fabs(m_cmd[0]-m_state[0]) > 1.6)
    {
        delta_x = boost::math::sign(m_cmd[0]-m_state[0]) * 1.6;
    }

    //put the coordinate transformation into polar coordinates with its origin at the goal position
    beta = angle(m_cmd[2], atan2(delta_y, delta_x));

    alpha = angle(m_state[2], atan2(delta_y, delta_x));

    beta1 = angle(m_cmd[2]+PI, atan2(delta_y, delta_x));

    alpha1 = angle(m_state[2]+ PI, atan2(delta_y, delta_x));

    dist = sqrt(delta_x*delta_x + delta_y* delta_y);

    k = -1/dist * (k2*(alpha-atan(-k1*beta)) + sin(alpha)*(1 + k1/(1+(k1*beta) * (k1*beta))));
    k_back = -1/dist * (k2*(alpha1-atan2(-k1*beta1,1)) + sin(alpha1)*(1 + k1/(1+(k1*beta1) * (k1*beta1))));
}

void 
motion_controller::shutdown()
{
    locomotor->shut_down();
    liftmotor->shut_down();
    printf("Shut down motor\n");
}




