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

  void messageCallback(const geometry_msgs::Twist::ConstPtr &msg);

  double r_ = .2;
  double r_center_ = .1;

  geometry_msgs::Twist incoming_msg;
  
  ros::Publisher pub_left;
  ros::Publisher pub_right;

  std_msgs::Float32 velLeft_;
  std_msgs::Float32 velRight_;

};

#endif // LOCOMOTION_MODULE_H
