/*
 *
 *      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the Stingray, iBotics nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "locomotion_module/locomotion_module.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "locom");
  ros::NodeHandle nh;

  // Create a new NodeExample object.
  LocomotionModule *locomotion_module = new LocomotionModule();

  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server
  // values can be overwritten.
  dynamic_reconfigure::Server<locomotion_module::locomotionModuleConfig> dr_srv;
  dynamic_reconfigure::Server<locomotion_module::locomotionModuleConfig>::CallbackType cb;
  cb = boost::bind(&LocomotionModule::configCallback, locomotion_module, _1, _2);
  dr_srv.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  
  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  //ros::NodeHandle pnh("~");
  //pnh.param("r", locomotion_module->r_, .1);
 
  ROS_INFO("r %f",locomotion_module->r_);
  // Create a publisher and name the topic.
  //use floats
  locomotion_module->pub_left = nh.advertise<oddbot_msgs::MotorCommand>("/object_3/motor/command", 10);
  locomotion_module->pub_right = nh.advertise<oddbot_msgs::MotorCommand>("/object_4/motor/command", 10);
  
  ros::Subscriber sub_message = nh.subscribe("cmd_vel", 1000, &LocomotionModule::velMessageCallback, locomotion_module);

  //ros::spin();
  // Main loop.
  double r_temp;
  double r_left;
  double r_right;	
  while (nh.ok())
   {
  //   // Publish the message. Do this in the callback for subscribing? 
     //ROS_INFO("R is %f",locomotion_module->r_);
     nh.getParamCached("/object_4/motor/dist_m",r_right);
     locomotion_module->r_right_ = r_right + locomotion_module->r_center_;
     nh.getParamCached("/object_3/motor/dist_m",r_left);
     locomotion_module->r_left_ = r_left + locomotion_module->r_center_;

     //ROS_INFO("r param is %f",r_temp);
     ros::spinOnce();
  //   //r.sleep();
   }

  return 0;
} // end main()
