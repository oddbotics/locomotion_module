#include "locomotion_module/locomotion_module.h"

LocomotionModule::LocomotionModule()
{
}

Locomotion::~LocomotionModule()
{
}

void LocomotionModule::publishMessage(ros::Publisher *pub_message, int side)
{
  //std_msgs::float32 msg;

  //pub_message->publish(msg);
}

void LocomotionModule::messageCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  //translate twist into left and right velocities
  velLeft_.data = msg.linear.x + msg.angular.z*r; 
  velRight_.data = msg.linear.x - msg.angular.z*r;
  //publish to left and right topics
  pub_left->publish(velLeft_);
  pub_right->publish(velRight_);
}

void LocomotionModule::configCallback(locomotion_module::locomotionModuleConfig &config, uint32_t level)
{
  r_ = config.r + r_center_;

}
