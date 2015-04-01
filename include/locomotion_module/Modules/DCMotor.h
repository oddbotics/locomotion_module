#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class DCMotor : public Module 
{
  public:
    DCMotor(int location);
    geometry_msgs::Twist getVelocity(){return this->motor_vel_mps;}
    std::string getType(){return "dc_motor";} 
    void calculateDesiredVelocity(double robot_linear, double robot_angular);  
    void publishDesiredVelocity();

  private:

    void motorCallback(const oddbot_msgs::ActuationFeedback::ConstPtr& msg);

    geometry_msgs::Twist motor_vel_mps;
    double des_motor_vel_mps;
    std::string frame;
    tf::Transform joint_base_tf,base_joint_tf;
    
    tf::TransformListener listener;
    ros::Publisher cmd_pub;
    ros::Subscriber fdbk_sub; 
    
};

#endif // DCMOTOR_H_
