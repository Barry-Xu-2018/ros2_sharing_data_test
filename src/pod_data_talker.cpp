// Copyright 2025 Sony Group Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "test_sharing_data/msg/test_data1m.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("pod_data_talker")
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    auto publish_message =
      [this]() -> void
      {
        count_++;
        auto loaned_msg = pod_pub_->borrow_loaned_message();

        *reinterpret_cast<uint32_t *>(&loaned_msg.get().data_array[0]) = count_;

        RCLCPP_INFO(this->get_logger(), "Publishing: %d", count_);
        pod_pub_->publish(std::move(loaned_msg));

      };

    pod_pub_ = this->create_publisher<test_sharing_data::msg::TestData1m>("chatter_pod", 10);
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  uint32_t count_ = 0;
  rclcpp::Publisher<test_sharing_data::msg::TestData1m>::SharedPtr pod_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
