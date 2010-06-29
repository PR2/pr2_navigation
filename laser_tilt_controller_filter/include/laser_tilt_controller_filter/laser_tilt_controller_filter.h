/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <XmlRpc.h>
#include <sensor_msgs/LaserScan.h>
#include <filters/filter_base.h>
#include <pr2_msgs/LaserScannerSignal.h>
#include <boost/thread/mutex.hpp>

namespace laser_tilt_controller_filter {
  
  /**
   * @class LaserTiltControllerFilter
   * @brief A filter that invalidates scans for certain periods of a tilt profile
   */
  class LaserTiltControllerFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {
    public:
      LaserTiltControllerFilter(): signal_received_(false){}

      bool loadFilterSignals(){
        XmlRpc::XmlRpcValue filter_sections;

        if(!filters::FilterBase<sensor_msgs::LaserScan>::getParam("filter_sections", filter_sections)){
          //its ok to have an empty filter sections list... but we'll just return
          return true;
        }

        if(!(filter_sections.getType() == XmlRpc::XmlRpcValue::TypeArray)){
          ROS_ERROR("The filter_sections must be a list of integers");
          return false;
        }

        //loop through the XmlRpc list
        for(int i = 0; i < filter_sections.size(); ++i){
          XmlRpc::XmlRpcValue section = filter_sections[i];

          //check to make sure that integer values are being used
          if(!(section.getType() == XmlRpc::XmlRpcValue::TypeInt)){
            ROS_ERROR("The tilt_profile_times must be a list of integers");
            return false;
          }

          double section_int = int(section);

          //push the time back onto our vector
          filter_signals_.push_back(section_int);
        }

        return true;
      }

      bool loadTiltProfileTiming(){
        XmlRpc::XmlRpcValue tilt_profile_times;
        if(!filters::FilterBase<sensor_msgs::LaserScan>::getParam("tilt_profile_times", tilt_profile_times)){
          ROS_ERROR("No tilt_profile_times were given");
          return false;
        }

        if(!(tilt_profile_times.getType() == XmlRpc::XmlRpcValue::TypeArray)){
          ROS_ERROR("The tilt_profile_times must be a list of doubles");
          return false;
        }

        if(tilt_profile_times.size() < 2){
          ROS_ERROR("The tilt_profile_times list must contain at least two values... a start and an end");
          return false;
        }

        //loop through the XmlRpc list
        for(int i = 0; i < tilt_profile_times.size(); ++i){
          XmlRpc::XmlRpcValue time_xml = tilt_profile_times[i];

          //check to make sure that integer/double values are being used
          if(!(time_xml.getType() == XmlRpc::XmlRpcValue::TypeInt || time_xml.getType() == XmlRpc::XmlRpcValue::TypeDouble)){
            ROS_ERROR("The tilt_profile_times must be a list of doubles");
            return false;
          }

          double time_dbl = time_xml.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(time_xml) : (double)(time_xml);

          //push the time back onto our vector
          tilt_profile_times_.push_back(time_dbl);
        }

        return true;
      }

      bool configure(){
        ROS_DEBUG("Filtering initialized");
        bool success = loadTiltProfileTiming() && loadFilterSignals();

        ros::NodeHandle n;
        signal_sub_ = n.subscribe("laser_tilt_controller/laser_scanner_signal", 1, &LaserTiltControllerFilter::signalCb, this);
        return success;
      }

      void signalCb(const pr2_msgs::LaserScannerSignal::ConstPtr& signal){
        boost::mutex::scoped_lock lock(mutex_);
        //we'll sync the timer to the signal for the beginning of the profile to make sure we don't drift
        if(signal->signal == 0){
          timer_zero_ = signal->header.stamp;
          signal_received_ = true;
        }
      }

      bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out){
        ROS_DEBUG("Signal filter running, current timer %.4f, signal_received: %d", timer_zero_.toSec(), signal_received_);
        //make sure that we've received at least one scanner signal before throwing out scans
        if(!signal_received_){
          scan_out = scan_in;
          return true;
        }

        boost::mutex::scoped_lock lock(mutex_);
        double time_diff = (scan_in.header.stamp - timer_zero_).toSec() + tilt_profile_times_.front();

        //make sure to put the time within the period of the profile
        ROS_DEBUG("time_diff: %.4f, periold: %.4f", time_diff, tilt_profile_times_.back());
        double period_time = fmod(time_diff, tilt_profile_times_.back());

        int profile_section = 0;
        //get the period in which the scan falls
        for(unsigned int i = 0; i < tilt_profile_times_.size() - 1; ++i){
          ROS_DEBUG("Checking period_time: %.4f against tilt_profile time: %.4f and next %.4f", 
              period_time, tilt_profile_times_[i], tilt_profile_times_[i+1]);
          if(period_time >= tilt_profile_times_[i] && period_time < tilt_profile_times_[i + 1]){
            profile_section = i;
          }
        }

        ROS_DEBUG("Profile section: %d, time: %.4f", profile_section, scan_in.header.stamp.toSec());

        //check if our current tilt signal is one that should be filtered
        for(unsigned int i = 0; i < filter_signals_.size(); ++i){
          if(filter_signals_[i] == profile_section){
            sensor_msgs::LaserScan empty_scan;
            empty_scan.header = scan_in.header;
            scan_out = empty_scan;
            ROS_DEBUG("Filtering out scan");
            return true;
          }
        }

        //if we made it here, then the tilt signal does not need to be
        //filtered, so we'll just send the scan on
        scan_out = scan_in;
        return true;
      }

      ~LaserTiltControllerFilter() {}

    private:
      std::vector<int> filter_signals_;
      std::vector<double> tilt_profile_times_;
      ros::Time timer_zero_;
      boost::mutex mutex_;
      bool signal_received_;
      ros::Subscriber signal_sub_;
  };

};
