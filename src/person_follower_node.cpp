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
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Twist.h"
#include "Scan.h"

#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <bica_core/Component.h>

#include <algorithm>
#include <vector>

namespace person_navigation
{
const float SIZE_X = 6.0;
const float SIZE_Y = 6.0;
const float RESOLUTION = 0.2;
const float CELL_SIZE_X = SIZE_X / RESOLUTION;
const float CELL_SIZE_Y = SIZE_Y / RESOLUTION;

const int UNKNOWN = 255;
const int OCCUPIED = 254;
const int FREE = 0;

const float TIME_TO_LOST = 5.0;

const double MAX_VX = 0.2;
const double MAX_VY = 0.2;
const double MAX_W = 0.3;

const float ROBOT_RADIOUS = 0.21;
const float PERSON_RADIOUS = 0.8;

const int INFL_CELLS = static_cast<int>(ROBOT_RADIOUS / RESOLUTION);

class PersonFollower : public bica::Component
{
public:
  PersonFollower()
    : nh_()
    , gradient_pub_(&nh_, &gradient_, "/base_footprint", "/gradient", true)
    , rate_counter_(0)
    , scan_("/pepper_robot/laser_corrected")
  // scan_("/scan_filtered")
  {
    gradient_.resizeMap(CELL_SIZE_X, CELL_SIZE_Y, RESOLUTION, -SIZE_X / 2, -SIZE_Y / 2);
    gradient_.setDefaultValue(UNKNOWN);
    gradient_.resetMap(0, 0, gradient_.getSizeInCellsX(), gradient_.getSizeInCellsY());

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  }

  void step()
  {
    tf::StampedTransform bf2person;

    if (!getLastTFPerson(bf2person))
      return;
    if (oldTF(bf2person))
    {
      ROS_WARN("Old TF to person");
      stop();
      return;
    }

    if (should_refresh_gradient())
    {
      reset_map();
      add_laserscan(bf2person);
      update_gradient(bf2person);

      gradient_pub_.publishCostmap();
    }

    geometry_msgs::Twist vel;

    calculateW(bf2person, vel);
    calculateV(bf2person, vel);

    double angle = atan2(bf2person.getOrigin().y(), bf2person.getOrigin().x());

    if (fabs(angle) > 0.78)  // 45deg
    {
      vel.linear.x /= 4.0;
      vel.linear.y /= 4.0;
    }
    else if (fabs(vel.angular.z) > 0.52)  // 30deg
    {
      vel.linear.x /= 2.0;
      vel.linear.y /= 2.0;
    }

    vel_pub_.publish(vel);
  }

  void deActivateCode()
  {
    stop();
  }

private:
  void clean_goal(const tf::StampedTransform& bf2person)
  {
    unsigned int cx, cy;
    if (gradient_.worldToMap(bf2person.getOrigin().x(), bf2person.getOrigin().y(), cx, cy))
    {
      for (int i = -INFL_CELLS * 2; i <= INFL_CELLS * 2; i++)
        for (int j = -INFL_CELLS * 2; j <= INFL_CELLS * 2; j++)
        {
          if (is_in_map(cx + i, cy + j))
            gradient_.setCost(cx + i, cy + j, FREE);
        }
    }
  }

  void calculateV(const tf::StampedTransform& bf2person, geometry_msgs::Twist& vel)
  {
    unsigned int ci, cj;
    gradient_.worldToMap(0, 0, ci, cj);

    int min_val = OCCUPIED;
    unsigned int mi, mj;
    for (int i = -2; i <= 2; i++)
      for (int j = -2; j <= 2; j++)
      {
        if (gradient_.getCost(ci + i, cj + j) < min_val)
        {
          mi = ci + i;
          mj = cj + j;
          min_val = gradient_.getCost(ci + i, cj + j);
        }
      }

    double x, y;
    gradient_.mapToWorld(mi, mj, x, y);

    vel.linear.x = std::max(std::min(x, MAX_VX), -MAX_VX);
    vel.linear.y = std::max(std::min(y, MAX_VY), -MAX_VY);
  }

  void calculateW(const tf::StampedTransform& bf2person, geometry_msgs::Twist& vel)
  {
    double angle = atan2(bf2person.getOrigin().y(), bf2person.getOrigin().x());

    vel.angular.z = std::max(std::min(angle, MAX_W), -MAX_W);
  }

  bool is_in_map(unsigned int i, unsigned int j)
  {
    return !(i < 0 || j < 0 || i > gradient_.getSizeInCellsX() || j > gradient_.getSizeInCellsY());
  }

  void set_cost(unsigned int i, unsigned int j, int value)
  {
    if (!is_in_map(i, j))
      return;

    int current_cost = gradient_.getCost(i, j);

    if (current_cost == OCCUPIED)
      return;

    if (current_cost <= value)
      return;

    gradient_.setCost(i, j, value);

    set_cost(i - 1, j, value + 5);
    set_cost(i, j - 1, value + 5);
    set_cost(i + 1, j, value + 5);
    set_cost(i, j + 1, value + 5);
  }

  void update_gradient(const tf::StampedTransform& bf2person)
  {
    unsigned int cx, cy;
    gradient_.worldToMap(bf2person.getOrigin().x(), bf2person.getOrigin().y(), cx, cy);

    set_cost(cx, cy, 0);
  }

  bool not_near(double x, double y, const tf::StampedTransform& bf2person)
  {
    double xr = bf2person.getOrigin().x();
    double yr = bf2person.getOrigin().y();

    double dist = sqrt((x - xr) * (x - xr) + (y - yr) * (y - yr));
    if (dist > PERSON_RADIOUS)
    {
      return true;
    }
    else
      return false;
  }

  void add_laserscan(const tf::StampedTransform& bf2person)
  {
    std::vector<tf::Stamped<tf::Point>> last_scan = scan_.getLastScan();

    std::vector<tf::Stamped<tf::Point>>::iterator it;
    for (it = last_scan.begin(); it != last_scan.end(); ++it)
    {
      unsigned int cx, cy;

      if (gradient_.worldToMap(it->getX(), it->getY(), cx, cy))
      {
        // gradient_.setCost(cx, cy, OCCUPIED);
        // Inflating
        if (not_near(it->getX(), it->getY(), bf2person))
        {
          for (int i = -INFL_CELLS; i <= INFL_CELLS; i++)
            for (int j = -INFL_CELLS; j <= INFL_CELLS; j++)
            {
              // gradient_.setCost(cx+i, cy+j, OCCUPIED);/*
              if (is_in_map(cx + i, cy + j))
                gradient_.setCost(cx + i, cy + j, OCCUPIED);
            }
        }
      }
    }
  }

  void reset_map()
  {
    gradient_.resetMap(0, 0, gradient_.getSizeInCellsX(), gradient_.getSizeInCellsY());
  }

  void stop()
  {
    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.angular.z = 0.0;

    vel_pub_.publish(vel);
  }

  bool should_refresh_gradient()
  {
    if (rate_counter_++ > 5)
    {
      rate_counter_ = 0;
      return true;
    }
    else
      return false;
  }

  bool oldTF(const tf::StampedTransform& bf2person)
  {
    return false;
    float diff = (ros::Time::now() - bf2person.stamp_).toSec();

    return (diff > TIME_TO_LOST);
  }

  bool getLastTFPerson(tf::StampedTransform& bf2person)
  {
    try
    {
      tf_listener_.lookupTransform("/base_footprint", "/follow_target", ros::Time(0), bf2person);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("getLastTFPerson\t%s", ex.what());
      return false;
    }

    double dist = bf2person.getOrigin().length();

    if (dist > SIZE_X / 2)
    {
      double norm = (1.0 / dist);
      bf2person.getOrigin() *= norm * ((SIZE_X - 0.1) / 2.0);
    }

    return true;
  }

  ros::NodeHandle nh_;

  costmap_2d::Costmap2D gradient_;
  costmap_2d::Costmap2DPublisher gradient_pub_;
  ros::Publisher vel_pub_;
  tf::TransformListener tf_listener_;

  Scan scan_;

  int rate_counter_;
};

};  // namespace person_navigation

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_follower");
  ros::NodeHandle n;

  person_navigation::PersonFollower follower;
  ros::Rate loop_rate(20);

  while (follower.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
