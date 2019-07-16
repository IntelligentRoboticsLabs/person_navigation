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
#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <pepper_basic_capabilities_msgs/EngageMode.h>
#include <pepper_basic_capabilities_msgs/ShowWeb.h>
#include <pepper_basic_capabilities_msgs/SetWebTag.h>
#include <topological_navigation_msgs/GetLocation.h>
#include <std_srvs/Empty.h>
#include <pepper_basic_capabilities_msgs/DoTalk.h>

#include <tf/tf.h>

TEST(TESTSuite, test_pddl1)
{
  ros::NodeHandle nh;
  ros::ServiceClient ready_pddl_srv = nh.serviceClient<std_srvs::Trigger>("/pddl_builder/update_domain");

  bool ready_srv = ready_pddl_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);

  if (ready_srv)
  {
    std_srvs::Trigger srv;
    EXPECT_TRUE(ready_pddl_srv.call(srv));
    EXPECT_TRUE(srv.response.success);

    std::string nums = "1";

    std::string command = "rosrun rosplan_planning_system popf /tmp/domain_person_navigation.pddl ";
    command = command + ros::package::getPath("person_navigation") + "/test/problem_test" + nums +
              ".pddl > /tmp/out_test" + nums + ".out";
    EXPECT_EQ(system(command.c_str()), 0);

    std::string comp_command = "diff  /tmp/out_test" + nums + ".out ";
    comp_command = comp_command + ros::package::getPath("person_navigation") + "/test/expected_out_test" + nums + ".ou"
                                                                                                                  "t";

    EXPECT_EQ(system(comp_command.c_str()), 0);
  }
}

TEST(TESTSuite, test_pddl2)
{
  ros::NodeHandle nh;
  ros::ServiceClient ready_pddl_srv = nh.serviceClient<std_srvs::Trigger>("/pddl_builder/update_domain");

  bool ready_srv = ready_pddl_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);

  if (ready_srv)
  {
    std_srvs::Trigger srv;
    EXPECT_TRUE(ready_pddl_srv.call(srv));
    EXPECT_TRUE(srv.response.success);

    std::string nums = "2";

    std::string command = "rosrun rosplan_planning_system popf /tmp/domain_person_navigation.pddl ";
    command = command + ros::package::getPath("person_navigation") + "/test/problem_test" + nums +
              ".pddl > /tmp/out_test" + nums + ".out";
    EXPECT_EQ(system(command.c_str()), 0);

    std::string comp_command = "diff  /tmp/out_test" + nums + ".out ";
    comp_command = comp_command + ros::package::getPath("person_navigation") + "/test/expected_out_test" + nums + ".ou"
                                                                                                                  "t";

    EXPECT_EQ(system(comp_command.c_str()), 0);
  }
}
TEST(TESTSuite, test_pddl3)
{
  ros::NodeHandle nh;
  ros::ServiceClient ready_pddl_srv = nh.serviceClient<std_srvs::Trigger>("/pddl_builder/update_domain");

  bool ready_srv = ready_pddl_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);

  if (ready_srv)
  {
    std_srvs::Trigger srv;
    EXPECT_TRUE(ready_pddl_srv.call(srv));
    EXPECT_TRUE(srv.response.success);

    std::string nums = "3";

    std::string command = "rosrun rosplan_planning_system popf /tmp/domain_person_navigation.pddl ";
    command = command + ros::package::getPath("person_navigation") + "/test/problem_test" + nums +
              ".pddl > /tmp/out_test" + nums + ".out";
    EXPECT_EQ(system(command.c_str()), 0);

    std::string comp_command = "diff  /tmp/out_test" + nums + ".out ";
    comp_command = comp_command + ros::package::getPath("person_navigation") + "/test/expected_out_test" + nums + ".ou"
                                                                                                                  "t";

    EXPECT_EQ(system(comp_command.c_str()), 0);
  }
}
TEST(TESTSuite, test_pddl4)
{
  ros::NodeHandle nh;
  ros::ServiceClient ready_pddl_srv = nh.serviceClient<std_srvs::Trigger>("/pddl_builder/update_domain");

  bool ready_srv = ready_pddl_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);

  if (ready_srv)
  {
    std_srvs::Trigger srv;
    EXPECT_TRUE(ready_pddl_srv.call(srv));
    EXPECT_TRUE(srv.response.success);

    std::string nums = "4";

    std::string command = "rosrun rosplan_planning_system popf /tmp/domain_person_navigation.pddl ";
    command = command + ros::package::getPath("person_navigation") + "/test/problem_test" + nums +
              ".pddl > /tmp/out_test" + nums + ".out";
    EXPECT_EQ(system(command.c_str()), 0);

    std::string comp_command = "diff  /tmp/out_test" + nums + ".out ";
    comp_command = comp_command + ros::package::getPath("person_navigation") + "/test/expected_out_test" + nums + ".ou"
                                                                                                                  "t";

    EXPECT_EQ(system(comp_command.c_str()), 0);
  }
}
TEST(TESTSuite, test_pddl5)
{
  ros::NodeHandle nh;
  ros::ServiceClient ready_pddl_srv = nh.serviceClient<std_srvs::Trigger>("/pddl_builder/update_domain");

  bool ready_srv = ready_pddl_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);

  if (ready_srv)
  {
    std_srvs::Trigger srv;
    EXPECT_TRUE(ready_pddl_srv.call(srv));
    EXPECT_TRUE(srv.response.success);

    std::string nums = "5";

    std::string command = "rosrun rosplan_planning_system popf /tmp/domain_person_navigation.pddl ";
    command = command + ros::package::getPath("person_navigation") + "/test/problem_test" + nums +
              ".pddl > /tmp/out_test" + nums + ".out";
    EXPECT_EQ(system(command.c_str()), 0);

    std::string comp_command = "diff  /tmp/out_test" + nums + ".out ";
    comp_command = comp_command + ros::package::getPath("person_navigation") + "/test/expected_out_test" + nums + ".ou"
                                                                                                                  "t";

    EXPECT_EQ(system(comp_command.c_str()), 0);
  }
}

TEST(TESTSuite, test_service_clients)
{
  ros::NodeHandle nh;
  ros::ServiceClient engage_srv =
      nh.serviceClient<pepper_basic_capabilities_msgs::EngageMode>("/pepper_basic_capabilities/engage_mode");
  ros::ServiceClient web_srv =
      nh.serviceClient<pepper_basic_capabilities_msgs::ShowWeb>("/pepper_basic_capabilities/show_tablet_web");
  ros::ServiceClient web_edit_srv =
      nh.serviceClient<pepper_basic_capabilities_msgs::SetWebTag>("/pepper_basic_capabilities/set_tag_web");
  ros::ServiceClient srv_goal_ =
      nh.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  ros::ServiceClient clear_cmap_srv = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  ros::ServiceClient message_srv =
      nh.serviceClient<pepper_basic_capabilities_msgs::DoTalk>("/pepper_basic_capabilities/talk", 1);

  bool ready_srv;

  ready_srv = engage_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);  //

  ready_srv = web_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);  //

  ready_srv = web_edit_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);  //

  ready_srv = srv_goal_.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);

  ready_srv = clear_cmap_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);  //

  ready_srv = message_srv.waitForExistence(ros::Duration(5.0));
  EXPECT_TRUE(ready_srv);  //
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_TopologicalNav");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
