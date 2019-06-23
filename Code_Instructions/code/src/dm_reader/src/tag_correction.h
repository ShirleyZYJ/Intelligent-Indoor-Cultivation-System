#include "dualmotor.h"
#include "singlemotor.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include "math.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>

class tag_correction
{
  public:
    tag_correction(void);
    bool init(ros::NodeHandle& nh);
    void running(void);
    void shutdown(void);

    enum MOVE
    {
        IDLE = 0,
        FORWARD = 1,
        BACKWARD = 2,
        LIFT =3,
        TURN = 4,
    };


  private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void absposeCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);

    ros::Subscriber m_pathSub;
    ros::Subscriber m_absposeSub;

    apriltags_ros::AprilTagDetectionArray tag_detections;
    apriltags_ros::AprilTagDetection::ConstPtr abs_pose;

    MOVE m_move;
    
    Eigen::Vector3d m_goal;
    Eigen::Vector3d m_cur;


    boost::shared_ptr<dualmotor> locomotor;
    boost::shared_ptr<singlemotor> liftmotor;

    geometry_msgs::PoseStamped::ConstPtr m_pose;
    nav_msgs::Path::ConstPtr m_path;


    std::pair <int,int> vel_loco;
    ros::Time cycle_start;
    double cycle_period;
    int count;
};
