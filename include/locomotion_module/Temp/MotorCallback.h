/* 
 * a class to hold the motor objects  
 */

#ifndef MOTOR_CALLBACK_H_
#define MOTOR_CALLBACK_H_

class MotorCallback 
{
  public:
    void operator()(const oddbot_msgs::ActuationFeedback::ConstPtr &msg){
      for(int i = 0; i < msg->item.size(); i++){
        if(msg->item[i].compare("cur_vel_mps") == 0){
          *motorVel = msg->value[i];
          return;
        }
      }
    }
    std::string getLocation(){return this->location;}
    double getVelocity(){return this->motor_vel_mps;}
    void setLocation(std::string location){this->location = location;}
    void setVelocity(double velocity)(this->motor_vel_mps = velocity;}

  private:
    std::string location; //maybe tf
    double motor_vel_mps;
    ros::Publisher cmd_pub;
    ros::Subscriber fdbk_sub; 

    //TF
    
};

#endif // MOTOR_CALLBACK_H_


