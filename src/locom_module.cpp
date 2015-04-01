#include "locomotion_module/locom_module.cpp"

LocomotionModule::LocomotionModule(){
  	//the main node handle
	ros::NodeHandle nh;

	//grab the parameters
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param<double>("num_connections", num_connections, 4);
	private_node_handle_.param<double>("connector_nums", connector_num, [1 2 3 4]);
	
	//set up the publishers and the subscribers
	robot_vel_pub = nh.advertise<geometry_msgs::Twist>(feedback, 1000);
	cmd_vel_sub = nh.subscribe(command, 1000, &LocomotionModule::updateDesiredVelocity, this);
	
	for(int i = 1; i <= this->num_connections; i++){
	  active_ports.push_back(false);
  }
}

//increment all of the locations and determine if they are active
void LocomotionModule::findActiveModules(){
  //increment through all connections
	for(int i = 1; i <= this->num_connections; i++)
		//see if it has been plugged in 
  	std::string loc("/connector_" + this->connector_num[i]);
  	std::string key;
	  if (ros::param::search(loc, key) && ~active_ports[i]){
	    //get the value
	    std::string val;
  	  ros::param::get(key, val);
  	  active_ports[i] = true;
 	    //see if it is a motor
 	    if(val.compare("dc_motor") == 0){
 	      DCMotor motor = DCMotor(this->connector_num[i]);
 	    	this->actuation.push_back(motor);
 	    	this->module_list.push_back(motor);
 	    }
	  }
	}
}

void LocomotionModule::updateDesiredVelocity(const geometry_msgs::Twist::ConstPtr &msg){
  //get the desired velocities
  //double robot_linear = msg->linear.x;
  //double robot_angular = msg->angular.z;

  //cycle through each module if it is a motor, calculate the motor desired motor velocity
  for(int i = 0; i < this->actuation.size(); i++){
  	actuation[i].calculateDesiredVelocity(&msg);

  }
  
  //call funtion to publish the desired motor velocities
  for(int i = 0; i < this->actuation.size(); i++){
  	actuation[i].publishDesiredVelocity();
  } 
}

void LocomotionModule::updateCurrentRobotVelocity(void){
  //get all of the motor velocities in the base_link frame
  //combine them to get the current robot velocity
  double linear = 0.0;
  double angular = 0.0;
  double num_motors = ((double)this->actuation.size());
  
  for(int i = 0; i < num_motors; i++){
    geometry_msgs::Twist velocity = actuation[i].getVelocity();
  	linear += velocity.linear.x;
  	angular += velocity.angular.z
  }
  
  linear = linear/num_motors;
  angular = angular/num_motors;
  
  // publish the current robot velocity  
  geometry_msgs::Twist robot_vel;
  odom_translated.linear.x = linear;
  odom_translated.angular.z = angular;
  odom_trans.publish(robot_vel);
}


int main(int argc char * argv[]){

  ros::init(argc, argv, "locomotion_module");
  
  //initialize the locomotion module (assume modules are plugged in and running)
  LocomotionModule locom = LocomotionModule();
  locom.findActiveModules();

  ros::Rate rate(30.0);
  while(ros::ok()){
    //check everything still connected or if new things connected (ping modules) (every 30s)
    
    //publish current Robot velocity
    locom.updateCurrentRobotVelocity();

    ros::spinOnce();
    rate.sleep();
  }

}
