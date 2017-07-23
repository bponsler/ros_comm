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
#include "std_msgs/msg/bool.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/chain.h"

using namespace message_filters;
typedef std_msgs::msg::Bool Msg;
typedef std_msgs::msg::Bool::SharedPtr MsgPtr;

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb(const MsgPtr)
  {
    ++count_;
  }

  int32_t count_;
};

TEST(Subscriber, simple)
{
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("simple");
  Helper h;
  Subscriber<Msg> sub(nh, "test_topic", 0);
  sub.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  rclcpp::publisher::Publisher<Msg>::SharedPtr pub = nh->create_publisher<Msg>("test_topic", 0);

  tf2::TimePoint start = tf2::get_now();
  while (h.count_ == 0 && (tf2::get_now() - start) < tf2::durationFromSec(1.0))
  {
    pub->publish(Msg());
    usleep(10000);
    rclcpp::spin_some(nh);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(Subscriber, subUnsubSub)
{
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("subUnsubSub");
  Helper h;
  Subscriber<Msg> sub(nh, "test_topic", 0);
  sub.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  rclcpp::publisher::Publisher<Msg>::SharedPtr pub = nh->create_publisher<Msg>("test_topic", 0);

  sub.unsubscribe();
  sub.subscribe();

  tf2::TimePoint start = tf2::get_now();
  while (h.count_ == 0 && (tf2::get_now() - start) < tf2::durationFromSec(1.0))
  {
    pub->publish(Msg());
    usleep(10000);
    rclcpp::spin_some(nh);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(Subscriber, subInChain)
{
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("subInChain");
  Helper h;
  Chain<Msg> c;
  c.addFilter(std::make_shared<Subscriber<Msg> >(nh, "test_topic", 0));
  c.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  rclcpp::publisher::Publisher<Msg>::SharedPtr pub = nh->create_publisher<Msg>("test_topic", 0);

  tf2::TimePoint start = tf2::get_now();
  while (h.count_ == 0 && (tf2::get_now() - start) < tf2::durationFromSec(1.0))
  {
    pub->publish(Msg());
    usleep(10000);
    rclcpp::spin_some(nh);
  }

  ASSERT_GT(h.count_, 0);
}

struct ConstHelper
{
  void cb(const MsgPtr msg)
  {
    msg_ = msg;
  }

  MsgPtr msg_;
};

struct NonConstHelper
{
  void cb(const MsgPtr msg)
  {
    msg_ = msg;
  }

  MsgPtr msg_;
};

TEST(Subscriber, singleNonConstCallback)
{
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("singleNonConstCallback");
  NonConstHelper h;
  Subscriber<Msg> sub(nh, "test_topic", 0);
  sub.registerCallback(&NonConstHelper::cb, &h);
  rclcpp::publisher::Publisher<Msg>::SharedPtr pub = nh->create_publisher<Msg>("test_topic", 0);
  MsgPtr msg(std::make_shared<Msg>());
  pub->publish(msg);

  rclcpp::spin_some(nh);

  ASSERT_TRUE(h.msg_ != NULL);
  //ASSERT_EQ(msg.get(), h.msg_.get()); // TODO: not sure this guaranteed to be true anymore
}

TEST(Subscriber, multipleNonConstCallbacksFilterSubscriber)
{
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared(
    "multipleNonConstCallbacksFilterSubscriber");
  NonConstHelper h, h2;
  Subscriber<Msg> sub(nh, "test_topic", 0);
  sub.registerCallback(&NonConstHelper::cb, &h);
  sub.registerCallback(&NonConstHelper::cb, &h2);
  rclcpp::publisher::Publisher<Msg>::SharedPtr pub = nh->create_publisher<Msg>("test_topic", 0);
  MsgPtr msg(std::make_shared<Msg>());
  pub->publish(msg);

  rclcpp::spin_some(nh);

  ASSERT_TRUE(h.msg_ != NULL);
  ASSERT_TRUE(h2.msg_ != NULL);
  EXPECT_NE(msg.get(), h.msg_.get());
  EXPECT_NE(msg.get(), h2.msg_.get());
  EXPECT_EQ(h.msg_.get(), h2.msg_.get());  // They are equal now
}

TEST(Subscriber, multipleCallbacksSomeFilterSomeDirect)
{
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared(
    "multipleCallbacksSomeFilterSomeDirect");
  NonConstHelper h, h2;
  Subscriber<Msg> sub(nh, "test_topic", 0);
  sub.registerCallback(&NonConstHelper::cb, &h);
  rclcpp::subscription::Subscription<Msg>::SharedPtr sub2 = nh->create_subscription<Msg>(
    "test_topic", 0, std::bind(&NonConstHelper::cb, &h2, std::placeholders::_1));
  rclcpp::publisher::Publisher<Msg>::SharedPtr pub = nh->create_publisher<Msg>("test_topic", 0);
  MsgPtr msg(std::make_shared<Msg>());
  pub->publish(msg);

  rclcpp::spin_some(nh);

  ASSERT_TRUE(h.msg_ != NULL);
  ASSERT_TRUE(h2.msg_ != NULL);
  EXPECT_NE(msg.get(), h.msg_.get());
  EXPECT_NE(msg.get(), h2.msg_.get());
  EXPECT_NE(h.msg_.get(), h2.msg_.get());
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("test_subscriber");

  return RUN_ALL_TESTS();
}


