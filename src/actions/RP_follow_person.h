/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */
#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>
#include <boost/foreach.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"

#include <pepper_basic_capabilities_msgs/DoTalk.h>
#include <pepper_basic_capabilities_msgs/ShowWeb.h>
#include <ir_planning/Action.h>
#include "actionlib/client/simple_action_client.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Bool.h"

#ifndef KCL_follow_person
#define KCL_follow_person

class RP_follow_person : public ir_planning::Action
{
public:
  explicit RP_follow_person(ros::NodeHandle& nh);

protected:
  void activateCode();
  void deActivateCode();
  void step();

private:
  void hriCallback(const std_msgs::String::ConstPtr& str_in);

  bool speech_on_;

  ros::NodeHandle nh_;
  std::string target_person_;

  ros::ServiceClient engage_srv_, web_srv;
  ros::Subscriber hri_sub;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
};

#endif
