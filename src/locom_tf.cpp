#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>

int main(int argc, char** argv){
  ros::init(argc, argv, "locomotion_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  double x,y,z, foot_x, foot_y, foot_z, c1_x,c1_y,c1_z;
  double roll,pitch,yaw, foot_r, foot_p, foot_y,c1_r,c1_p,c1_y;

  std::vector<tf::Vector3> positions;
  std::vector<tf::Quaternion> oritentations;
  std::vector<std::string> locations;
  std::vector<int> connector_num
  
  std::string connector = ros::this_node::getNamespace();

  //read in a private node handle to determine frame
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<double>("/footprint/x", foot_x, 0.0);
  private_node_handle_.param<double>("/footprint/y", foot_y, 0.0);
  private_node_handle_.param<double>("/footprint/z", foot_z, 0.0);
  private_node_handle_.param<double>("/footprint/roll", foot_r, 0.0);
  private_node_handle_.param<double>("/footprint/pitch", foot_p, 0.0);
  private_node_handle_.param<double>("/footprint/yaw", foot_y, 0.0);
  private_node_handle_.param<double>("/connector_1/x", c1_x, 0.0);
  private_node_handle_.param<double>("/connector_1/y", c1_y, 0.0);
  private_node_handle_.param<double>("/connector_1/z", c1_z, 0.0);
  private_node_handle_.param<double>("/connector_1/roll", c1_r, 0.0);
  private_node_handle_.param<double>("/connector_1/pitch", c1_p, 0.0);
  private_node_handle_.param<double>("/connector_1/yaw", c1_y, 0.0);
  private_node_handle_.param<double>("number_connections", num_connections, 4);  
  private_node_handle_.param<double>("connector_nums", connector_num, [2 3 4 5]);

  for(int i = 1; i <= num_connections; i++){
	  std::string connector("connector_" + connector_num[i]);  
	  ros::NodeHandle private_node_handle_("~/connection_"+i);
	  private_node_handle_.param<double>("x", x, 0.0);
	  private_node_handle_.param<double>("y", y, 0.0);
	  private_node_handle_.param<double>("z", z, 0.0);
	  private_node_handle_.param<double>("roll", roll, 0.0);
	  private_node_handle_.param<double>("pitch", pitch, 0.0);
	  private_node_handle_.param<double>("yaw", yaw, 0.0);
	
	  tf::Vector3 vect(x, y, z);
	  tf::Quaternion quat;
	  quat.setRPY(roll, pitch, yaw);

	  positions.push_back(vect);
    orientations.push_back(quat);	
	  locations.push_back(connector);
  }

  tf::Vector3 foot_pos(foot_x, foot_y, foot_z);
  tf::Quaternion foot_quat;
  foot_quat.setRPY(foot_r, foot_p, foot_y);
  
  tf::Vector3 c1_pos(c1_x, c1_y, c1_z);
  tf::Quaternion c1_quat;
  c1_quat.setRPY(c1_r, c1_p, c1_y);

  ros::Rate rate(10.0);
  while (node.ok()){
    //publish base_link to all of the connectors
    for(int i = 0; i < locations.size(); i++){
      transform.setOrigin( positions[i]  );
      transform.setRotation( orientations[i] );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link" ,location[i]));
    }    

    //publish base_footprint to the base_link
    transform.setOrigin(foot_pos);
    transform.setRotation( foot_quat );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint" , "base_link"));
    //publish base_link to the top connector 1
    transform.setOrigin(foot_pos);
    transform.setRotation( foot_quat );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link" , "connector_1"));
    rate.sleep();
  }
  return 0;
};
