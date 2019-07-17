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
/*
 * Scan.h
 *
 *  Created on: 24/12/2015
 *      Author: paco
 */

#ifndef SCAN_H_
#define SCAN_H_

#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf/tf.h>

#include <string>
#include <vector>

namespace person_navigation
{
class Scan
{
public:
  explicit Scan(std::string laser_topic);
  virtual ~Scan();

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

  const std::vector<tf::Stamped<tf::Point> >& getLastScan() const
  {
    return scan_bf_;
  };

private:
  ros::NodeHandle nh_;

  std::string baseFrameId_;
  std::string laser_topic_;

  tf::TransformListener tfListener_;

  std::vector<tf::Stamped<tf::Point> > scan_bf_;
  double robot_radious_;

  tf::MessageFilter<sensor_msgs::LaserScan>* tfScanSub_;
  message_filters::Subscriber<sensor_msgs::LaserScan>* scanSub_;
};

};  // namespace person_navigation

#endif /* SCAN_H_ */
