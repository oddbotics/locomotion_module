/**
 * node class for locomotion module

 */

#ifndef LOCOM_MODULE_H
#define LOCOM_MODULE_H

 
// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include "oddbot_msgs/ActuationCommand.h"
#include "oddbot_msgs/ActuationFeedback.h"
#include <cmath> 
#include <string>
#include "locomotion_module/Modules/DCMotor.h"
#include "locomotion_module/Modules/Module.h"

// Dynamic reconfigure includes.
//#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
//#include <locomotion_module/locomotionModuleConfig.h>


class LocomotionModule
{
  public:
   LocomotionModule();
   ~LocomotionModule();
   void findActiveModules();
   void updateCurrentRobotVelocity();   

  protected:
  
    void updateDesiredVelocity(const geometry_msgs::Twist::ConstPtr &msg);
    void updateOdometry(void);
    
    int num_connections;
    std::vector<int> connector_num;
    std::vector<DCMotor*> actuation;
    std::vector<bool> active_ports;
    //std::vector<Module> module_list; 
    
    tf::TransformListener listener;
    ros::Publisher robot_vel_pub;
    ros::Subscriber cmd_vel_sub;
};

#endif // LOCOM_MODULE_H
