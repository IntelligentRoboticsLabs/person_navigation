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
 * Scan.cpp
 *
 *  Created on: 24/12/2015
 *      Author: paco
 */

#include "Scan.h"

#include <string>

namespace person_navigation
{
Scan::Scan(std::string laser_topic)
  : nh_()
  , baseFrameId_("base_footprint")
  , laser_topic_(laser_topic)
  ,  // /pepper_robot/laser_corrected -> Pepper /scan
  scan_bf_()
{
  scanSub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, laser_topic_, 5);
  tfScanSub_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scanSub_, tfListener_, baseFrameId_, 5);
  tfScanSub_->registerCallback(boost::bind(&Scan::scanCallback, this, _1));
}

Scan::~Scan()
{
}

void Scan::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  float o_t_min, o_t_max, o_t_inc;

  o_t_min = scan_in->angle_min;
  o_t_max = scan_in->angle_max;
  o_t_inc = scan_in->angle_increment;

  int num_points = static_cast<int>(2.0 * o_t_max / o_t_inc);

  tf::Stamped<tf::Point> scan_sensor[num_points];
  scan_bf_.resize(num_points);

  float rx = 0.0, ry = 0.0;
  int c = 0;

  for (int i = 0; i < num_points; i++)
  {
    float theta = o_t_min + i * o_t_inc;
    float r = scan_in->ranges[i];

    scan_sensor[i].setX(r * cos(theta));
    scan_sensor[i].setY(r * sin(theta));
    scan_sensor[i].setZ(0.0);
    scan_sensor[i].setW(1.0);
    scan_sensor[i].stamp_ = scan_in->header.stamp;
    scan_sensor[i].frame_id_ = scan_in->header.frame_id;

    tfListener_.transformPoint(baseFrameId_, scan_sensor[i], scan_bf_[i]);
  }
}

};  // namespace person_navigation
