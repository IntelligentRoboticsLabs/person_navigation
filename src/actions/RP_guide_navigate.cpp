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
#include "RP_guide_navigate.h"

#include <string>
#include <vector>

/* The implementation of RP_guide_navigate.h */

/* constructor */
RP_guide_navigate::RP_guide_navigate(ros::NodeHandle& nh)
  : nh_(nh), Action("guide_navigate"), action_client_("/move_base", false), goal()
{
  sonar_sub = nh.subscribe("/pepper_robot/sonar/back", 1, &RP_guide_navigate::sonarCallback, this);
  head_sub = nh.subscribe("/pepper_robot/head_touch", 1, &RP_guide_navigate::headTouchCallback, this);
  stop_action_ = nh_.subscribe("/action/guide_navigate/stop", 1, &RP_guide_navigate::stopActionCB, this);
  message_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::DoTalk>("/pepper_basic_capabilities/talk", 1);
  srv_goal_ = nh.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  clear_cmap_srv = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  web_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::ShowWeb>("/pepper_basic_capabilities/show_tablet_web");
  engage_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::EngageMode>("/pepper_basic_capabilities/engage_mode");
  guide_started = false;
  guide_move_paused = false;
  personInRange = true;
  headTouched = false;
  state = INIT;
}

void RP_guide_navigate::talk(std::string s)
{
  pepper_basic_capabilities_msgs::DoTalk srv;
  srv.request.sentence = s;
  message_srv.call(srv);
}

void RP_guide_navigate::attentionOn()
{
  pepper_basic_capabilities_msgs::EngageMode engage_msg_;
  engage_msg_.request.mode = "off";
  engage_srv.call(engage_msg_);
}

void RP_guide_navigate::headTouchCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& touch_in){
  if(touch_in->state==1){
    headTouched = true;
    ROS_INFO("[headTouchCallback ]HEAD TOUCHED");
  }
}

void RP_guide_navigate::stopActionCB(const std_msgs::Empty::ConstPtr& msg)
{
  setFail();
}


void RP_guide_navigate::sonarCallback(const sensor_msgs::Range::ConstPtr& sonar_in){
    ROS_DEBUG("[sonarCallback] Range -- %f",sonar_in->range);
    if(sonar_in->range < 1.5 && sonar_in->range > sonar_in->min_range){
        t = ros::Time::now();
        personInRange = true;
    }else if(ros::Time::now() > t + ros::Duration(3)){
        personInRange = false;
    }
}

void RP_guide_navigate::activateCode()
{
  tfListener_.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(2.0));
  tfListener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), odom2bf);

  while(!action_client_.waitForServer(ros::Duration(5.0))){
      ROS_WARN("[guide_navigate] Waiting for the move_base action server to come up");
  }
  std::string wpID;
  bool found = false;
  for(size_t i=0; i<last_msg_.parameters.size(); i++) {
      if(0==last_msg_.parameters[i].key.compare("wp2")) {
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

  ROS_DEBUG("[guide_navigate]Commanding to %lf %lf", goal_pose_.pose.position.x, goal_pose_.pose.position.y);

  //move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = goal_pose_;
  goal.target_pose.header.frame_id = "/map";
  headTouched = false;
  state = INIT;
}

void RP_guide_navigate::deActivateCode()
{
  action_client_.cancelAllGoals();
}

void RP_guide_navigate::step()
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
        talk("¡Sígueme!");
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
      else if(headTouched){
        action_client_.cancelAllGoals();
        talk("De acuerdo, te espero aquí. ¡No tardes, por favor!");
        headTouched = false;
        state = PAUSED;
      }
      break;
    case RUNNING:
      ROS_INFO("State RUNNING");
      if(!personInRange){
          action_client_.cancelAllGoals();
          talk("Parece que te he perdido, colocate detrás de mi.");
          state = WAITING;
      }else if(headTouched){
          action_client_.cancelAllGoals();
          talk("De acuerdo, te espero aquí. ¡No tardes, por favor!");
          headTouched = false;
          state = PAUSED;
      }
      break;
    case WAITING:
      ROS_INFO("State WAITING");
      if(personInRange){
          talk("Seguimos con el paseo. No te alejes mucho");
          state = RUNNING;
          goal.target_pose.header.stamp = ros::Time::now();
          action_client_.sendGoal(goal);
      }else
      action_client_.cancelAllGoals();
      break;
    case PAUSED:
      ROS_INFO("State PAUSED");
      if(headTouched){
          talk("¡Hola otra vez! Seguimos con el paseo, mantente detrás de mi");
          headTouched = false;
          state = RUNNING;
          goal.target_pose.header.stamp = ros::Time::now();
          action_client_.sendGoal(goal);
      }
      break;
  }
  bool finished_before_timeout = action_client_.waitForResult(ros::Duration(0.5));
  if (finished_before_timeout) {
      actionlib::SimpleClientGoalState state = action_client_.getState();
      ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

      if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
          guide_started = false;
          setSuccess();
          return;
      } else {
          remove_current_goal();
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
  ros::init(argc, argv, "rosplan_interface_guide_navigate");
  ros::NodeHandle nh("~");

  std::string actionserver;
  nh.param("action_server", actionserver, std::string("/move_base"));

  // create PDDL action subscriber
  RP_guide_navigate rpmb(nh);

  // listen for action dispatch
  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}