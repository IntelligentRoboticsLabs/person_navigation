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
#include "RP_approach_person.h"

/* constructor */
RP_approach_person::RP_approach_person(ros::NodeHandle& nh) : nh_(nh), Action("approach_person", 20)
{
  engage_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::EngageMode>("/pepper_basic_capabilities/engage_mode");
  web_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::ShowWeb>("/pepper_basic_capabilities/show_tablet_web");
  web_edit_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::SetWebTag>("/pepper_basic_capabilities/set_tag_web");
}

void RP_approach_person::activateCode()
{
  /* HRI tablet */
  pepper_basic_capabilities_msgs::ShowWeb w_srv;
  w_srv.request.url = "common/approaching.html";
  web_srv.call(w_srv);

  bool found = false;
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    if (0 == last_msg_.parameters[i].key.compare("p"))
    {
      target_person_ = last_msg_.parameters[i].value;
    }
  }

  addDependency("people_detector");
  addDependency("person_follower_node");

  pepper_basic_capabilities_msgs::EngageMode engage_msg_;
  engage_msg_.request.mode = "fully";
  engage_srv.call(engage_msg_);

  starting_ts_ = ros::Time::now();
}

void RP_approach_person::deActivateCode()
{
  removeDependency("people_detector");
  removeDependency("person_follower_node");
}

void RP_approach_person::step()
{
  tf::StampedTransform map2person, bf2map, cam2map;
  tf::Transform bf2person, cam2person;
  try
  {
    tf_listener_.lookupTransform("/base_footprint", "/map", ros::Time(0), bf2map);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("*1* %s", ex.what());
    return;
  }
  try
  {
    tf_listener_.lookupTransform("/map", target_person_, ros::Time(0), map2person);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("21* %s", ex.what());
    return;
  }
  try
  {
    tf_listener_.lookupTransform("/CameraTop_frame", "/map", ros::Time(0), cam2map);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("*3* %s", ex.what());
    return;
  }

  bf2person = bf2map * map2person;

  tf::StampedTransform bf2person_st(bf2person, ros::Time::now(), "/base_footprint", "/follow_target");
  double angle = atan2(bf2person.getOrigin().y(), bf2person.getOrigin().x());

  if ((ros::Time::now() - map2person.stamp_).toSec() < 5.0)
  {
    try
    {
      tf_broadcaster_.sendTransform(bf2person_st);
    }
    catch (tf::TransformException& exception)
    {
      ROS_ERROR("%s", exception.what());
    }
  }

  cam2person = cam2map * map2person;
  double dist = cam2person.getOrigin().length();
  ROS_INFO("RP: dist: %lf\tangle: %lf", dist, angle);

  /* Edit info tablet */
  pepper_basic_capabilities_msgs::SetWebTag we_srv;
  we_srv.request.tag = "*say*";
  we_srv.request.value = std::to_string(dist);
  web_edit_srv.call(we_srv);
  /* HRI tablet */
  pepper_basic_capabilities_msgs::ShowWeb w_srv;
  w_srv.request.url = "common/approaching.html";
  web_srv.call(w_srv);

  if ((dist < 1.5 && fabs(angle) < 0.1) || ((ros::Time::now() - starting_ts_) > ros::Duration(30.0)))
    setSuccess();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_approach_person_to");
  ros::NodeHandle nh("~");

  RP_approach_person rpmb(nh);

  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}
