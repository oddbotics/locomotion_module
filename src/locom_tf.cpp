#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>

int main(int argc, char** argv){
  ros::init(argc, argv, "locomotion_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  double x,y,z, foot_x, foot_y, foot_z, c1_x,c1_y,c1_z;
  double roll,pitch,yaw, foot_r, foot_p, foot_yaw,c1_r,c1_p,c1_yaw;
  int num_connections;

  std::vector<double> pos_x,pos_y,pos_z;
  std::vector<double> r,p,ya;
  std::vector<std::string> locations;
  std::vector<int> connector_num;  
  
  std::string connector = ros::this_node::getNamespace();

  //read in a private node handle to determine frame
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param<double>("footprint/x", foot_x, 0.0);
  private_node_handle_.param<double>("footprint/y", foot_y, 0.0);
  private_node_handle_.param<double>("footprint/z", foot_z, 0.0);
  private_node_handle_.param<double>("footprint/roll", foot_r, 0.0);
  private_node_handle_.param<double>("footprint/pitch", foot_p, 0.0);
  private_node_handle_.param<double>("footprint/yaw", foot_yaw, 0.0);
  private_node_handle_.param<double>("connector_1/x", c1_x, 0.0);
  private_node_handle_.param<double>("connector_1/y", c1_y, 0.0);
  private_node_handle_.param<double>("connector_1/z", c1_z, 0.0);
  private_node_handle_.param<double>("connector_1/roll", c1_r, 0.0);
  private_node_handle_.param<double>("connector_1/pitch", c1_p, 0.0);
  private_node_handle_.param<double>("connector_1/yaw", c1_yaw, 0.0);
  private_node_handle_.param<int>("number_connections", num_connections, 4);  

  private_node_handle_.param<std::vector<int>>("connector_nums", connector_num, {2, 3, 4, 5});

  for(int i = 0; i < num_connections; i++){
	  std::string connector("connector_" +std::to_string(connector_num[i]));  
	  ROS_INFO("GETTING %s",connector.c_str());
	  ros::NodeHandle private_node_handle_("~");
	  private_node_handle_.param<double>(connector+"/x", x, 0.0);
	  private_node_handle_.param<double>(connector+"/y", y, 0.0);
	  private_node_handle_.param<double>(connector+"/z", z, 0.0);
	  private_node_handle_.param<double>(connector+"/roll", roll, 0.0);
	  private_node_handle_.param<double>(connector+"/pitch", pitch, 0.0);
	  private_node_handle_.param<double>(connector+"/yaw", yaw, 0.0);
		
	  ROS_INFO("(%f,%f,%f) :: (%f,%f,%f)",x,y,z,roll,pitch,yaw);	
	  pos_x.push_back(x);
	  pos_y.push_back(y);
	  pos_z.push_back(z);
	  r.push_back(roll);
	  p.push_back(pitch);
	  ya.push_back(yaw);
	  locations.push_back(connector);
  }

  tf::Vector3 foot_pos(foot_x, foot_y, foot_z);
  tf::Quaternion foot_quat;
  foot_quat.setRPY(foot_r, foot_p, foot_yaw);
  
  tf::Vector3 c1_pos(c1_x, c1_y, c1_z);
  tf::Quaternion c1_quat;
  c1_quat.setRPY(c1_r, c1_p, c1_yaw);

  ros::Rate rate(30.0);
  while (node.ok()){
    //publish base_link to all of the connectors
    for(int i = 0; i < locations.size(); i++){
      transform.setOrigin(tf::Vector3(pos_x[i],pos_y[i],pos_z[i]));
      tf::Quaternion quat;
      quat.setRPY(r[i],p[i],ya[i]);
      transform.setRotation(quat);
      br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "base_link" ,locations[i]));
    }    

    //publish base_footprint to the base_link
    transform.setOrigin(foot_pos);
    transform.setRotation( foot_quat );
    br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "base_footprint" , "base_link"));
    //publish base_link to the top connector 1
    transform.setOrigin(c1_pos);
    transform.setRotation( c1_quat );
    br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "base_link" , "connector_1"));
    rate.sleep();
  }
  return 0;
};
