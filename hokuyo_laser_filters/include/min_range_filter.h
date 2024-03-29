/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASER_SCAN_MIN_RANGE_FILTER_H
#define LASER_SCAN_MIN_RANGE_FILTER_H
/**
\author Addisu Taddese
@b ScanMinRangeFilter removes values less than min_range and replaces them with max_range

**/


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace hokuyo_laser_filters
{

class LaserScanMinRangeFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

  bool configure()
  {
    return true;
  }

  virtual ~LaserScanMinRangeFilter()
  {

  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan ;
    //ROS_INFO("Update");

    // Check every reading and change those below range_min and above range_max to be slightly below range_max
    for (unsigned int i=0; i < input_scan.ranges.size(); i++){
        if (filtered_scan.ranges[i] >= input_scan.range_max || filtered_scan.ranges[i] <= input_scan.range_min)
        {
            filtered_scan.ranges[i] = input_scan.range_max - 1e-4;
        }
    }
    return true;
  }
} ;

}

#endif // LASER_SCAN_MIN_RANGE_FILTER_H
