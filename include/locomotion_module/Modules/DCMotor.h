#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "ros/ros.h"
#include "locomotion_module/Modules/Module.h"
#include "geometry_msgs/Twist.h"
#include "oddbot_msgs/ActuationCommand.h"
#include "oddbot_msgs/ActuationFeedback.h"
#include "tf/transform_listener.h"

class DCMotor : public Module 
{
  public:
    DCMotor(int location);
    geometry_msgs::Twist getVelocity(){return this->motor_vel_mps;}
    std::string getType(){return "dc_motor";} 
    void calculateDesiredVelocity(double x, double y, double wz);  
    void publishDesiredVelocity();

  private:

    void motorCallback(const oddbot_msgs::ActuationFeedback::ConstPtr& msg);

    geometry_msgs::Twist motor_vel_mps;
    double des_motor_vel_mps;
    std::string frame;
    tf::StampedTransform joint_base_tf,base_joint_tf;
    
    tf::TransformListener listener;
    ros::Publisher cmd_pub;
    ros::Subscriber fdbk_sub; 
    
};

#endif // DCMOTOR_H_
