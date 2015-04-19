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
	
	for(int i = 0; i < this->num_connections; i++){
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
  
  ROS_INFO("-------------------Checking--------------------");
  //increment through all connections
  for(int i = 0; i < this->num_connections; i++){
    //see if it has been plugged in 
    std::string loc("connector_" + std::string(std::to_string(this->connector_num[i]) + "/type"));
    ROS_INFO("LOOKING FOR %s, CURRENT STATUS %s",loc.c_str(),(this->active_ports[i])?"true":"false");
    std::string key;
    if (ros::param::search(loc, key) && !this->active_ports[i]){
      //get the value
      std::string val;
      ros::param::get(key, val);
      //active_port[i] = true;
      //see if it is a dc motor
      if(val.compare("dc_motor") == 0){
        //try to get the transform before added
	try {
	  std::string from = std::string("connector_" + std::to_string(this->connector_num[i]));
	  std::string to = std::string("dc_motor_" + std::string(std::to_string(this->connector_num[i]) + "/wheel"));
	  tf::StampedTransform transform;
	  listener.lookupTransform(from,to,ros::Time(0),transform);
	  DCMotor * motor = new DCMotor(this->connector_num[i]);
     	  this->actuation.push_back(motor);
	  this->active_ports[i] = true;
	  ROS_INFO("MODULE AT CONNECTOR %d ADDED: at actuation: %lu", this->connector_num[i], this->actuation.size());
    	} catch (tf::TransformException ex){
          ROS_INFO("PARAM THERE BUT NO TF");
//	  ros::param::del(key);
	}	  
      }
    } //else if(this->active_ports[i]){
      //get the Motor
      //ROS_INFO("Module at %s", std::string(std::to_string(this->connector_num[i])).c_str());
      //for(int m = 0; m < actuation.size(); m++){
	//if(actuation
      /*int loc;
      bool found = false;
      for(int m = 0; m < actuation.size(); m++){
	if(actuation[m]->getPort() == this->connector_num[i]){
          loc = m;
	  found = true;
	}
      }
      //get the type
      if(found){
        if(actuation[loc]->getType().compare("dc_motor") == 0){
          //try to get the transform
          try {
	    std::string from = std::string("connector_" + std::to_string(this->connector_num[i]));
            std::string to = std::string("dc_motor_" + std::string(std::to_string(this->connector_num[i]) + "/wheel"));
            tf::StampedTransform transform;
            listener.lookupTransform(from,to,ros::Time(0),transform);
            ROS_INFO("MODULE STILL AT CONNECTOR %d", this->connector_num[i]);
          } catch (tf::TransformException ex){
            ROS_INFO("MODULE AT CONNECTOR %d NO LONGER THERE",this->connector_num[i]);
	    ROS_INFO("REMOVING");
	    active_ports[i] = false;
	    delete actuation[loc];
  	    this->actuation.erase(this->actuation.begin()+(loc-1));
	    ros::param::del(key);
          }         
        }
      }
    }*/
  }
  for(int m = 0; m < actuation.size(); m++){
    if((ros::Time::now().toSec() - actuation[m]->getLastReport()) > 5.0){
      ROS_INFO("MODULE AT CONNECTOR %d NO LONGER THERE",actuation[m]->getPort());
      ROS_INFO("REMOVING");
      for(int i = 0; i < active_ports.size(); i++){
        if(this->connector_num[i] == actuation[m]->getPort()){
        active_ports[i] = false;
        }
      }
      delete actuation[m];
      this->actuation.erase(this->actuation.begin()+m);
      std::string key;
      std::string loc("connector_" + std::string(std::to_string(actuation[m]->getPort()) + "/type"));
      ros::param::search(loc,key);
//      ros::param::del(key);
    }
  }
  ROS_INFO("-----------------------FINISHED CHECKING --------------------");
}

void LocomotionModule::updateDesiredVelocity(const geometry_msgs::Twist::ConstPtr& msg){
  static bool isForward = true;
  //get the desired velocities
  //double robot_linear = msg->linear.x;
  //double robot_angular = msg->angular.z;

//  geometry_msgs::Twist des_robot_vel = *msg;

  //cycle through each module if it is a motor, calculate the motor desired motor velocity
//  ROS_INFO("GOT COMMANDlinear: %f, angular %f",msg->linear.x,msg->angular.z);
  for(int i = 0; i < this->actuation.size(); i++){
  	actuation[i]->calculateDesiredVelocity(msg->linear.x,msg->linear.y,msg->angular.z);
//	ROS_INFO("Calculating Velocities");
  }
  
  ROS_INFO("Size %ld, ISFORWARD %s",this->actuation.size(),(isForward)?"true":"false");
  //call funtion to publish the desired motor velocities
  if(isForward){
    for(int i = 0; i < this->actuation.size(); i++){
          actuation[i]->publishDesiredVelocity();
	  ROS_INFO("Publishing Velocities Forward");
    }
    isForward = false;
  } else {
    for(int i = this->actuation.size() - 1; i < 0; i--){
          actuation[i]->publishDesiredVelocity();
          ROS_INFO("Publishing Velocities Reverse");
    }
    isForward = true;
  }
}

void LocomotionModule::updateCurrentRobotVelocity(void){
  //get all of the motor velocities in the base_link frame
  //combine them to get the current robot velocity
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular = 0.0;
  int num_motors = this->actuation.size();
  
  for(int i = 0; i < num_motors; i++){
    geometry_msgs::Twist velocity = actuation[i]->getVelocity();
  	linear_x += velocity.linear.x;
	linear_y += velocity.linear.y;
  	angular += velocity.angular.z;
  }
  
  linear_y = linear_y/((double)num_motors);
  linear_x = linear_x/((double)num_motors);
  angular = angular/((double)num_motors);
  
  // publish the current robot velocity  
  geometry_msgs::Twist robot_vel;
  robot_vel.linear.x = linear_x;
  robot_vel.linear.y = linear_y;
  robot_vel.angular.z = angular;
  robot_vel_pub.publish(robot_vel);
}


int main(int argc, char * argv[]){

  ros::init(argc, argv, "locomotion_module");
  
  //initialize the locomotion module (assume modules are plugged in and running)
  LocomotionModule * locom = new LocomotionModule();
  locom->findActiveModules();
  double last_check = ros::Time::now().toSec();

  ros::Rate rate(30.0);
  while(ros::ok()){
    if(ros::Time::now().toSec() - last_check > 10.0){
    	//check everything still connected or if new things connected (ping modules) (every 10s)
    	locom->findActiveModules();
	last_check = ros::Time::now().toSec();
    }    

    //publish current Robot velocity
    locom->updateCurrentRobotVelocity();

    ros::spinOnce();
    rate.sleep();
  }

}
