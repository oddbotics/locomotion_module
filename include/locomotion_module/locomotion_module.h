/**
 * node class for locomotion module

 */

#ifndef LOCOMOTION_MODULE_H
#define LOCOMOTION_MODULE_H


// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "oddbot_msgs/MotorCommand.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <locomotion_module/locomotionModuleConfig.h>


class LocomotionModule
{
 public:
  //! Constructor
  LocomotionModule();

  ~LocomotionModule();

  void configCallback(locomotion_module::locomotionModuleConfig &config, uint32_t level);

  void publishMessage(ros::Publisher *pub_message, int side);

  void velMessageCallback(const geometry_msgs::Twist::ConstPtr &msg);

  void odomMessageCallback(const geometry_msgs::Twist::ConstPtr &msg);

  double r_ = .5;
  double r_center_ = .1;

  geometry_msgs::Twist incoming_msg;
  
  ros::Publisher pub_left;
  ros::Publisher pub_right;

  ros::Publisher odom_pub;
  //tf::TransformBroadcaster odom_broadcaster;
  double dt;
  double delta_x;
  double delta_y;
  double delta_th;

  double x;
  double y;
  double th;

  oddbot_msgs::MotorCommand velLeft_;
  oddbot_msgs::MotorCommand velRight_;
  //std_msgs::Float32 velLeft_;
  //std_msgs::Float32 velRight_;

  ros::Time current_time, last_time;
  



};

#endif // LOCOMOTION_MODULE_H
