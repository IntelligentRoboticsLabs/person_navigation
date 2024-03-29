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
#include "sensor_msgs/Range.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <bica_planning/Action.h>
#include <bica_graph/graph_client.h>
#include <topological_navigation_msgs/GetLocation.h>

#ifndef KCL_cross
#define KCL_cross

class RP_guide_cross : public bica_planning::Action
{
public:
  explicit RP_guide_cross(ros::NodeHandle& nh);

protected:
  void activateCode();
  void deActivateCode();
  void step();

private:
  ros::NodeHandle nh_;

  ros::Timer timer;
  enum StateType
  {
    DOOR_OPENED,
    DOOR_CLOSED,
    UNKNOWN
  };
  StateType state;
  std::string actionserver_, sonar_topic_, sonar_frame_, robot_id;
  geometry_msgs::PoseStamped goal_pose_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
  ros::ServiceClient srv_goal_, clear_cmap_srv;
  ros::Subscriber sonar_sub;
  move_base_msgs::MoveBaseGoal goal;
  bool goal_sended, door_msg_sended, sonar_activate;
  tf::TransformListener tf_listener_;
  tf::MessageFilter<sensor_msgs::Range>* tf_sonar_sub_;
  message_filters::Subscriber<sensor_msgs::Range>* sonar_sub_;
  bica_graph::GraphClient graph_;

  void sonarCallback(const sensor_msgs::Range::ConstPtr& sonar_in);
};

#endif
