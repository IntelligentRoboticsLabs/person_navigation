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
#include "RP_guide_cross.h"

#include <string>
#include <vector>

/* The implementation of RP_guide_cross.h */

/* constructor */
RP_guide_cross::RP_guide_cross(ros::NodeHandle& nh)
  : nh_(nh), Action("guide_cross"), action_client_("/move_base", false)
{
  nh_.param<std::string>("sonar_topic", sonar_topic_, "sonar");
  nh_.param<std::string>("frame_id", sonar_frame_, "");

  if (sonar_frame_ != "")
  {
    sonar_sub_ = new message_filters::Subscriber<sensor_msgs::Range> (nh_, sonar_topic_, 5);
    tf_sonar_sub_ = new tf::MessageFilter<sensor_msgs::Range> (*sonar_sub_, tf_listener_, sonar_frame_, 5);
    tf_sonar_sub_ -> registerCallback(boost::bind(&RP_guide_cross::sonarCallback, this, _1));
  }
  else
    sonar_sub = nh.subscribe(sonar_topic_, 1, &RP_guide_cross::sonarCallback, this);


  srv_goal_ = nh_.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  clear_cmap_srv = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
}

void RP_guide_cross::activateCode()
{
  while (!action_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("[guide_cross] Waiting for the move_base action server to come up");
  }
  sonar_activate = true;
  goal_sended = false;
  door_msg_sended = false;

  state = UNKNOWN;
  std::string wpID;
  bool found = false;
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("wp2"))
    {
      wpID = last_msg_.parameters[i].value;
      found = true;
    }
    else if(0 == last_msg_.parameters[i].key.compare("r"))
    {
      robot_id = last_msg_.parameters[i].value;
    }
  }
  std::vector<boost::shared_ptr<geometry_msgs::Pose> > results;
  topological_navigation_msgs::GetLocation srv;
  srv.request.waypoint = wpID;
  if (srv_goal_.call(srv))
  {
    results.push_back(boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose(srv.response.position)));
  }
  else
  {
    ROS_ERROR("Failed to call service /topological_nav/get_location");
    setFail();
    return;
  }

  goal_pose_.pose = *(results[0]);

  ROS_INFO("[guide_cross]Commanding to [%s] (%f %f)", wpID.c_str(), goal_pose_.pose.position.x,
           goal_pose_.pose.position.y);
  goal.target_pose = goal_pose_;
  goal.target_pose.header.frame_id = "map";
}

void RP_guide_cross::deActivateCode()
{
  action_client_.cancelAllGoals();
}

void RP_guide_cross::sonarCallback(const sensor_msgs::Range::ConstPtr& sonar_in)
{
  if (!isActive())
    return;

  // ROS_INFO("[sonarCallback] Range -- %f",sonar_in->range);
  if (sonar_activate && (UNKNOWN || DOOR_CLOSED) && sonar_in->range > 0.7 && sonar_in->range > sonar_in->min_range)
  {
    sonar_activate = false;
    state = DOOR_OPENED;
  }
  else if (sonar_activate && UNKNOWN && sonar_in->range < 0.7 && sonar_in->range > sonar_in->min_range)
  {
    state = DOOR_CLOSED;
  }
}

/* action dispatch callback */
void RP_guide_cross::step()
{
  std::string speech_msg;
  switch (state)
  {
    case DOOR_OPENED:
      if (!goal_sended)
      {
        graph_.add_edge(robot_id, "say: Crossing the door.", robot_id);
        goal.target_pose.header.stamp = ros::Time::now();
        std_srvs::Empty srv;
        if (!clear_cmap_srv.call(srv))
        {
          ROS_ERROR("Failed to call service /move_base/clear_costmaps");
          return;
        }
        action_client_.sendGoal(goal);
        goal_sended = true;
      }
      else
      {
        bool finished_before_timeout = action_client_.waitForResult(ros::Duration(0.5));
        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = action_client_.getState();
          ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());
          if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            setSuccess();
            return;
          }
          else
          {
            setFail();
            return;
          }
        }
      }
      break;
    case DOOR_CLOSED:
      if (!door_msg_sended)
      {
        graph_.add_edge(robot_id, "say: Could you open the door, please?.", robot_id);
        door_msg_sended = true;
      }
      break;
    case UNKNOWN:
      break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_cross");
  ros::NodeHandle nh("~");

  std::string actionserver;
  nh.param("action_server", actionserver, std::string("/move_base"));

  RP_guide_cross rpmb(nh);

  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}
