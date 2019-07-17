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
#include "RP_guide_move.h"

#include <string>
#include <vector>

/* The implementation of RP_guide_move.h */

/* constructor */
RP_guide_move::RP_guide_move(ros::NodeHandle& nh)
  : nh_(nh), Action("guide_move"), action_client_("/move_base", false), goal()
{
  nh_.param<std::string>("sonar_topic", sonar_topic_, "sonar");
  sonar_sub = nh.subscribe(sonar_topic_, 1, &RP_guide_move::sonarCallback, this);
  srv_goal_ = nh.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  guide_started = false;
  guide_move_paused = false;
  personInRange = true;
  state = DIRECTIONS;
}

void RP_guide_move::sonarCallback(const sensor_msgs::Range::ConstPtr& sonar_in)
{
  if (!isActive())
    return;

  ROS_DEBUG("[sonarCallback] Range -- %f", sonar_in->range);
  if (sonar_in->range < 1.5 && sonar_in->range > sonar_in->min_range)
  {
    t = ros::Time::now();
    personInRange = true;
  }
  else
  {
    if (ros::Time::now() > t + ros::Duration(3))
    {
      personInRange = false;
    }
  }
}

void RP_guide_move::activateCode()
{
  tfListener_.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(2.0));
  tfListener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), odom2bf);

  while(!action_client_.waitForServer(ros::Duration(5.0))){
      ROS_WARN("[guide_move] Waiting for the move_base action server to come up");
  }
  std::string wpID;
  bool found = false;
  for(size_t i=0; i<last_msg_.parameters.size(); i++) {
      if(0==last_msg_.parameters[i].key.compare("to")) {
          wpID = last_msg_.parameters[i].value;
          found = true;
      }
  }

  std::vector< boost::shared_ptr<geometry_msgs::Pose> > results;
  topological_navigation_msgs::GetLocation srv;
  srv.request.waypoint = wpID;
  if (srv_goal_.call(srv)){
      results.push_back(boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose(srv.response.position)));
  }else{
      ROS_ERROR("Failed to call service /topological_nav/get_location");
      setFail();
      return;
  }

  goal_pose_.pose = *(results[0]);

  ROS_DEBUG("[guide_move]Commanding to %lf %lf", goal_pose_.pose.position.x, goal_pose_.pose.position.y);

  //move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = goal_pose_;
  goal.target_pose.header.frame_id = "/map";
  state = INIT;
}

void RP_guide_move::deActivateCode()
{
  action_client_.cancelAllGoals();
}

void RP_guide_move::step()
{
  tfListener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), odom2bfnow);
  float hOrigin = sqrt(pow(odom2bf.getOrigin().x(),2)+pow(odom2bf.getOrigin().y(),2));
  float hNow = sqrt(pow(odom2bfnow.getOrigin().x(),2)+pow(odom2bfnow.getOrigin().y(),2));
  std_srvs::Empty srv_;

  switch(state){
      case INIT:
        if(guide_started){
            goal.target_pose.header.stamp = ros::Time::now();
            action_client_.sendGoal(goal);
            state = STARTING;
        } else {
          ROS_INFO("[guide_move] INIT state");
          // talk("¡Sígueme!");
          goal.target_pose.header.stamp = ros::Time::now();
          action_client_.sendGoal(goal);
          state = STARTING;
        }
        break;
      case STARTING:
        ROS_INFO("State STARTING");
        guide_started = true;
        if(hNow > hOrigin + 0.5)
          state = RUNNING;
        break;
      case RUNNING:
        ROS_INFO("State RUNNING");
        if(!personInRange)
        {
          action_client_.cancelAllGoals();
          // talk("Parece que te he perdido, colocate detrás de mi.");
          state = WAITING;
        }
        break;
      case WAITING:
        ROS_INFO("State WAITING");
        if(personInRange){
          goal.target_pose.header.stamp = ros::Time::now();
          action_client_.sendGoal(goal);
          // talk("Seguimos con el paseo. No te alejes mucho");
          state = RUNNING;
        }else
          action_client_.cancelAllGoals();
        break;
    }
  bool finished_before_timeout = action_client_.waitForResult(ros::Duration(0.5));
  if (finished_before_timeout) {
      actionlib::SimpleClientGoalState state = action_client_.getState();
      ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          guide_started = false;
          setSuccess();
          return;
      }
      else if(state == actionlib::SimpleClientGoalState::ABORTED)
      {
          setFail();
          return;
      }
  }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_guide_move");
  ros::NodeHandle nh("~");

  std::string actionserver;
  nh.param("action_server", actionserver, std::string("/move_base"));

  // create PDDL action subscriber
  RP_guide_move rpmb(nh);

  // listen for action dispatch
  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}
