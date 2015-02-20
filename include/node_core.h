/**
 *  \file node__core.h
 *  \brief Common class functions for example talker and listener nodes.
 */

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
 *      Edited by Eric.
 */

#ifndef E_NODE_CORE_H
#define E_NODE_CORE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
//#include "node_example/NodeExampleData.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <node_core/nodeCoreConfig.h>

class NodeCore
{
public:
  //! Constructor.
  NodeCore();

  //! Destructor.
  ~NodeCore();

  //! Callback function for dynamic reconfigure server.
  void configCallback(node_core::nodeCoreConfig &config, uint32_t level);

  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message);

  //! Callback function for subscriber.
  void messageCallback(const node_example::NodeExampleData::ConstPtr &msg);

  double r;

  geometry_msgs::Twist incoming_msg;
  
  float velLeft;
  float velRight;

};

#endif // E_NODE_CORE_H
