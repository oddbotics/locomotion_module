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
  velLeft_.des_ctrl = msg->linear.x + msg->angular.z*r_left_; 
  velRight_.des_ctrl = msg->linear.x - msg->angular.z*r_right_;
  //publish to left and right topics
  pub_left.publish(velLeft_);
  pub_right.publish(velRight_);
  ROS_INFO("r_left: %f :: r_right: %f",r_left_,r_right_);
}

void LocomotionModule::updateVelocities(const oddbot_msgs::ActuationFeedback::ConstPtr &msg, double * motorVel){
  for(int i = 0; i < msg->item.size(); i++){
    if(msg->item[i].compare("cur_vel_mps") == 0){
      *motorVel = msg->value[i];
      return;
    }
  }
}

void LocomotionModule::updateLeftVelocities(const oddbot_msgs::ActuationFeedback::ConstPtr &msg){
  for(int i = 0; i < msg->item.size(); i++){
    if(msg->item[i].compare("cur_vel_mps") == 0){
      receivedLeftVel_ = msg->value[i];
      return;
    }
  }
}

void LocomotionModule::updateRightVelocities(const oddbot_msgs::ActuationFeedback::ConstPtr &msg){
  for(int i = 0; i < msg->item.size(); i++){
    if(msg->item[i].compare("cur_vel_mps") == 0){
      receivedRightVel_ = msg->value[i];
      return;
    }
  }
}

void LocomotionModule::odomUpdateCallback(const std_msgs::Float32::ConstPtr &msg)
{
  //might have to change this if the two motors don't publish enough
  //right now this assumes that listening for one is enough
  receivedLeftVel_ = msg->data;


}

void LocomotionModule::odomTranslationCallback(const std_msgs::Float32::ConstPtr &msg)
{
  //might have to change this if the two motors don't publish enough
  //right now this assumes that listening for one is enough
  //and that both will be updating at about the same time
  receivedRightVel_ = msg->data;
  double combinedVel = receivedRightVel_ + receivedLeftVel_;
  geometry_msgs::Twist odom_translated;
  odom_translated.linear.x = combinedVel/2;
  odom_translated.angular.z = receivedRightVel_ - combinedVel/2;
  odom_trans.publish(odom_translated);

}

void LocomotionModule::publishOdomTranslationCallback()
{
  double combinedVel = receivedRightVel_ + receivedLeftVel_;
  geometry_msgs::Twist odom_translated;
  odom_translated.linear.x = combinedVel/2;
  odom_translated.angular.z = (receivedRightVel_ - combinedVel/2)/r_right_;
  odom_trans.publish(odom_translated);

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
    //odom_broadcaster.sendTransform(odom_trans);

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
    odom.twist.twist.linear.x = msg->linear.x;
    odom.twist.twist.linear.y = msg->linear.y;
    odom.twist.twist.angular.z = msg->angular.z;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
  


}



void LocomotionModule::configCallback(locomotion_module::locomotionModuleConfig &config, uint32_t level)
{
  r_ = config.r + r_center_;
 // ROS_INFO("R changed %f",r_);

}
