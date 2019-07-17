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
#include "RP_follow_person.h"

#include <string>

/* constructor */
RP_follow_person::RP_follow_person(ros::NodeHandle& nh) : nh_(nh), speech_on_(false), Action("follow_person", 20)
{
  hri_sub = nh_.subscribe("/pepper_perception/hri_comm", 1, &RP_follow_person::hriCallback, this);
  web_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::ShowWeb>("/pepper_basic_capabilities/how_tablet_web");
}

void RP_follow_person::hriCallback(const std_msgs::String::ConstPtr& str_in)
{
  if (!isActive())
    return;

  if (str_in->data.find("stop") != std::string::npos &&
      (str_in->data.find("follow") != std::string::npos || str_in->data.find("following") != std::string::npos))
  {
    setSuccess();
  }
}

void RP_follow_person::activateCode()
{
  /* HRI tablet */
  pepper_basic_capabilities_msgs::ShowWeb w_srv;
  w_srv.request.url = "helpmecarry/stopfollowing.html";
  web_srv.call(w_srv);

  bool found = false;
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    if (0 == last_msg_.parameters[i].key.compare("p"))
    {
      target_person_ = last_msg_.parameters[i].value;
    }
  }

  addDependency("person_follower_node");
  addDependency("sound_detector");
}

void RP_follow_person::deActivateCode()
{
  removeDependency("person_follower_node");
  removeDependency("sound_detector");
}

void RP_follow_person::step()
{
  tf::StampedTransform map2person, bf2map;
  tf::Transform bf2person;
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
    ROS_ERROR("*1* %s", ex.what());
    return;
  }

  bf2person = bf2map * map2person;

  tf::StampedTransform bf2person_st(bf2person, ros::Time::now(), "/base_footprint", "/follow_target");

  double dist = bf2person.getOrigin().length();
  double angle = atan2(bf2person.getOrigin().y(), bf2person.getOrigin().x());

  ROS_DEBUG("RP: dist: %lf\tangle: %lf", dist, angle);

  try
  {
    tf_broadcaster_.sendTransform(bf2person_st);
  }
  catch (tf::TransformException& exception)
  {
    ROS_ERROR("%s", exception.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_follow_person");
  ros::NodeHandle nh("~");

  RP_follow_person rpmb(nh);

  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}
