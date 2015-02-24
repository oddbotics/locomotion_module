#include "locomotion_module/locomotion_module.h"

LocomotionModule::LocomotionModule()
{
}

LocomotionModule::~LocomotionModule()
{
}

void LocomotionModule::publishMessage(ros::Publisher *pub_message, int side)
{
  //std_msgs::float32 msg;

  //pub_message->publish(msg);
}

void LocomotionModule::velMessageCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  //translate twist into left and right velocities
  velLeft_.data = msg->linear.x + msg->angular.z*r_; 
  velRight_.data = msg->linear.x - msg->angular.z*r_;
  //publish to left and right topics
  pub_left.publish(velLeft_);
  pub_right.publish(velRight_);
}

void LocomotionModule::odomMessageCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  current_time = ros::Time::now();

  dt = (current_time - last_time).toSec();
  delta_th = msg->angular.z*dt;
  delta_x = (msg->linear.x*cos(th) )* dt;
  delta_y = (msg->linear.y*sin(th) )* dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
  


}



void LocomotionModule::configCallback(locomotion_module::locomotionModuleConfig &config, uint32_t level)
{
  r_ = config.r + r_center_;
  ROS_INFO("R changed %f",r_);

}
