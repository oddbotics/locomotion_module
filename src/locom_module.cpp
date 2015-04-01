#include "locomotion_module/locom_module.h"

LocomotionModule::LocomotionModule(){
  	//the main node handle
	ros::NodeHandle nh;

	//grab the parameters
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param<int>("num_connections", num_connections, 4);
	private_node_handle_.param<std::vector<int>>("connector_nums", connector_num, {2, 3, 4, 5});
	
	//set up the publishers and the subscribers
	robot_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_vel", 1000);
	cmd_vel_sub = nh.subscribe("/cmd_vel", 1000, &LocomotionModule::updateDesiredVelocity, this);
	
	for(int i = 1; i <= this->num_connections; i++){
	  active_ports.push_back(false);
  }
}

LocomotionModule::~LocomotionModule(){
  for(int i = 0; i < this->actuation.size(); i++){
    delete actuation[i];
  }
}

//increment all of the locations and determine if they are active
void LocomotionModule::findActiveModules(){
  //increment through all connections
  for(int i = 1; i <= this->num_connections; i++){
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
        DCMotor * motor = new DCMotor(this->connector_num[i]);
     	this->actuation.push_back(motor);
//     	this->module_list.push_back(motor);
      }
    }
  }
}

void LocomotionModule::updateDesiredVelocity(const geometry_msgs::Twist::ConstPtr &msg){
  //get the desired velocities
  //double robot_linear = msg->linear.x;
  //double robot_angular = msg->angular.z;

//  geometry_msgs::Twist des_robot_vel = *msg;

  //cycle through each module if it is a motor, calculate the motor desired motor velocity
  for(int i = 0; i < this->actuation.size(); i++){
  	actuation[i]->calculateDesiredVelocity(*msg);
  }
  
  //call funtion to publish the desired motor velocities
  for(int i = 0; i < this->actuation.size(); i++){
  	actuation[i]->publishDesiredVelocity();
  } 
}

void LocomotionModule::updateCurrentRobotVelocity(void){
  //get all of the motor velocities in the base_link frame
  //combine them to get the current robot velocity
  double linear = 0.0;
  double angular = 0.0;
  int num_motors = this->actuation.size();
  
  for(int i = 0; i < num_motors; i++){
    geometry_msgs::Twist velocity = actuation[i]->getVelocity();
  	linear += velocity.linear.x;
  	angular += velocity.angular.z;
  }
  
  linear = linear/((double)num_motors);
  angular = angular/((double)num_motors);
  
  // publish the current robot velocity  
  geometry_msgs::Twist robot_vel;
  robot_vel.linear.x = linear;
  robot_vel.angular.z = angular;
  robot_vel_pub.publish(robot_vel);
}


int main(int argc, char * argv[]){

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
