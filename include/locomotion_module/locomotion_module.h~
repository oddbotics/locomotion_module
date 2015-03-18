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
#include "oddbot_msgs/ActuationCommand.h"
#include "oddbot_msgs/ActuationFeedback.h"
#include <cmath> 
#include <string>

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

  void odomTranslationCallback(const std_msgs::Float32::ConstPtr &msg);

  void odomUpdateCallback(const std_msgs::Float32::ConstPtr &msg);

  void odomMessageCallback(const geometry_msgs::Twist::ConstPtr &msg);

  void updateVelocities(const oddbot_msgs::ActuationFeedback::ConstPtr &msg, double * motorVel);

  void updateLeftVelocities(const oddbot_msgs::ActuationFeedback::ConstPtr &msg);
  void updateRightVelocities(const oddbot_msgs::ActuationFeedback::ConstPtr &msg);

  void publishOdomTranslationCallback();

  double r_ = .5;
  double r_center_ = .1;
  double r_right_;
  double r_left_;
  geometry_msgs::Twist incoming_msg;
  
  ros::Publisher pub_left;
  ros::Publisher pub_right;

  ros::Publisher odom_trans;
  ros::Publisher odom_pub;
  //tf::TransformBroadcaster odom_broadcaster;
  double dt;
  double delta_x;
  double delta_y;
  double delta_th;

  double x;
  double y;
  double th;

  double receivedLeftVel_;
  double receivedRightVel_;

  oddbot_msgs::ActuationCommand velLeft_;
  oddbot_msgs::ActuationCommand velRight_;
  //std_msgs::Float32 velLeft_;
  //std_msgs::Float32 velRight_;

  ros::Time current_time, last_time;
  



};

#endif // LOCOMOTION_MODULE_H
