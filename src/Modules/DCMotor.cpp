#include "locomotion_module/Modules/DCMotor.h"

DCMotor::DCMotor(int location) : Module(location){
	
	ros::NodeHandle nh;

	frame = std::string("dc_motor_" + std::string(std::to_string(location) + "/wheel"));
	
	try{
		listener.lookupTransform(this->frame, "base_link",  
				                   ros::Time(0), joint_base_tf);
	} catch (tf::TransformException ex){
	}
	
	try{
		listener.lookupTransform("base_link", this->frame,  
				                   ros::Time(0), base_joint_tf);
	} catch (tf::TransformException ex){
	}
	
	std::string feedback("/connector_" + std::string(std::to_string(location) + "/dc_motor/feedback"));
	std::string command("/connector_" + std::string(std::to_string(location) + "/dc_motor/command"));
	
	cmd_pub = nh.advertise<oddbot_msgs::ActuationCommand>(command, 1000);
	fdbk_sub = nh.subscribe(feedback, 1000, &DCMotor::motorCallback, this);

}

void DCMotor::motorCallback(const oddbot_msgs::ActuationFeedback::ConstPtr& msg){
  double motorVel;
  for(int i = 0; i < msg->item.size(); i++){
    if(msg->item[i].compare("cur_vel_mps") == 0){
      motorVel = msg->value[i];
      break;
    }
  }
  
  try{
		listener.lookupTransform("/base_link", this->frame,  
				                   ros::Time(0), base_joint_tf);
	} catch (tf::TransformException ex){
	}
	
  //calculate robot velocity using transform
  double x = base_joint_tf.getOrigin().x();
	double y = base_joint_tf.getOrigin().y();
	double r = sqrt(pow(x,2) + pow(y,2)) * (x*y)/fabs(x*y);
  motor_vel_mps.linear.x = motorVel;
  motor_vel_mps.angular.z = motorVel/r;
}

void DCMotor::calculateDesiredVelocity(double x, double y, double wz){
  ROS_INFO("frame %s", this->frame.c_str());
	try{
		listener.lookupTransform("base_link", this->frame,  
				                   ros::Time(0), base_joint_tf);
	} catch (tf::TransformException ex){
	  ROS_INFO("NO TF RECEIVED");
	}
  ROS_INFO("GOT COMMANDlinear: %f, angular %f",x,wz);	
  this->des_motor_vel_mps = 0.0;
	double ox = base_joint_tf.getOrigin().x();
	double oy = base_joint_tf.getOrigin().y();
	double r = sqrt(pow(ox,2) + pow(oy,2)) * (ox*oy)/fabs(ox*oy);

	if (ox < 0.00001 || ox > -0.00001){
 		ox = 0.0;
		r = sqrt(pow(ox,2) + pow(oy,2)) * (oy)/fabs(oy);
	} else if (oy < 0.00001 || oy > -0.00001){
		oy = 0.0;
		r = sqrt(pow(ox,2) + pow(oy,2)) * (ox)/fabs(ox);
	}

	ROS_INFO("tf info: x: %f, y: %f, r: %f",ox,oy,r);
	this->des_motor_vel_mps += y*cos(atan2(oy,ox));
	this->des_motor_vel_mps += -1*x*sin(atan2(oy,ox));
	this->des_motor_vel_mps += wz*r;
	ROS_INFO("calced vel %f", this->des_motor_vel_mps);
}

void DCMotor::publishDesiredVelocity(){
  oddbot_msgs::ActuationCommand msg;
  msg.des_ctrl = this->des_motor_vel_mps;
  cmd_pub.publish(msg);
  ROS_INFO("PUBLISHED COMMAND: %f", msg.des_ctrl);
}
