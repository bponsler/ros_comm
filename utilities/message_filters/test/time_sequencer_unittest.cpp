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

#include <gtest/gtest.h>

#include "tf2/time.h"
#include "message_filters/time_sequencer.h"

using namespace message_filters;

struct Header
{
  tf2::TimePoint stamp;
};


struct Msg
{
  Header header;
  int data;
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

namespace ros
{
namespace message_traits
{
template<>
struct TimeStamp<Msg>
{
  static tf2::TimePoint value(const Msg& m)
  {
    return m.header.stamp;
  }
};
}
}

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb(const MsgConstPtr&)
  {
    ++count_;
  }

  int32_t count_;
};

TEST(TimeSequencer, simple)
{
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("simple");
  
  TimeSequencer<Msg> seq(tf2::durationFromSec(1.0), tf2::durationFromSec(0.01), 10);
  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, _1));
  MsgPtr msg(boost::make_shared<Msg>());
  msg->header.stamp = tf2::get_now();
  seq.add(msg);

  usleep(100000);
  rclcpp::spin_some(nh);
  ASSERT_EQ(h.count_, 0);

  //ros::Time::setNow(ros::Time::now() + ros::Duration(2.0));

  usleep(100000);
  rclcpp::spin_some(nh);

  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSequencer, compilation)
{
  TimeSequencer<Msg> seq(tf2::durationFromSec(1.0), tf2::durationFromSec(0.01), 10);
  TimeSequencer<Msg> seq2(tf2::durationFromSec(1.0), tf2::durationFromSec(0.01), 10);
  seq2.connectInput(seq);
}

struct EventHelper
{
public:
  void cb(const ros::MessageEvent<Msg const>& evt)
  {
    event_ = evt;
  }

  ros::MessageEvent<Msg const> event_;
};

TEST(TimeSequencer, eventInEventOut)
{
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("simple");
    
  TimeSequencer<Msg> seq(tf2::durationFromSec(1.0), tf2::durationFromSec(0.01), 10);
  TimeSequencer<Msg> seq2(seq, tf2::durationFromSec(1.0), tf2::durationFromSec(0.01), 10);
  EventHelper h;
  seq2.registerCallback(&EventHelper::cb, &h);

  ros::MessageEvent<Msg const> evt(boost::make_shared<Msg const>(), tf2::get_now());
  seq.add(evt);

  //ros::Time::setNow(ros::Time::now() + ros::Duration(2));
  while (!h.event_.getMessage())
  {
    usleep(10000);
    rclcpp::spin_some(nh);
  }

  EXPECT_EQ(h.event_.getReceiptTime(), evt.getReceiptTime());
  EXPECT_EQ(h.event_.getMessage(), evt.getMessage());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("time_sequencer_test");;
  //ros::Time::setNow(ros::Time());

  return RUN_ALL_TESTS();
}


