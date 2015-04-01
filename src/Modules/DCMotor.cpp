DCMotor::DCMotor(int location) : Module(location, "dc_motor"){

	frame = "dc_motor_" + location + "/wheel";
	
	try{
		listener.lookupTransform(this->frame, "/base_link",  
				                   ros::Time(0), joint_base_tf);
	} catch (tf::TransformException ex){
	}
	
	try{
		listener.lookupTransform("/base_link", this->frame,  
				                   ros::Time(0), base_joint_tf);
	} catch (tf::TransformException ex){
	}
	
	cmd_pub = nh.advertise<oddbot_msgs::ActuationCommand>(feedback, 1000);
	fdbk_sub = nh.subscribe(command, 1000, &DCMotor::motorCallback, this);

}

void DCMotor::motorCallback(const oddbot_msgs::ActuationFeedback::ConstPtr& msg){
  double motorVel
  for(int i = 0; i < msg->item.size(); i++){
    if(msg->item[i].compare("cur_vel_mps") == 0){
      motorVel = msg->value[i];
      break;
    }
  }
  
  try{
		listener.lookupTransform("/base_link", this->frame(),  
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

void calculateDesiredVelocity(geometry_msgs::Twist des_robot_vel){
  try{
		listener.lookupTransform(this->frame, "/base_link",  
				                   ros::Time(0), joint_base_tf);
	} catch (tf::TransformException ex){
	}
	this->des_motor_vel_mps = 0.0;
	double x = base_joint_tf.getOrigin().x();
	double y = base_joint_tf.getOrigin().y();
	double r = sqrt(pow(x,2) + pow(y,2)) * (x*y)/fabs(x*y);
	this->des_motor_vel_mps += des_robot_vel.linear.y*cos(atan2(y,x));
	this->des_motor_vel_mps += des_robot_vel.linear.x*sin(atan2(y,x));
	this->des_motor_vel_mps += des_robot_vel.angular.z*r;
}

void DCMotor::publishDesiredVelocity(
	oddbot_msgs::ActuationCommand msg;
	msg.des_ctrl = this->des_motor_vel_mps;
  cmd_pub.publish(msg);
);
