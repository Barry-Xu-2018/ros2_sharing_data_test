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

#include "rclcpp/rclcpp.hpp"

#include "test_sharing_data/msg/test_data1m.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node("pod_data_listener")
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](const test_sharing_data::msg::TestData1m::SharedPtr msg) -> void
      {
        uint32_t data = *reinterpret_cast<uint32_t *>(&msg->data_array[0]);
        RCLCPP_INFO(this->get_logger(), "Received : %d", data);
      };
    sub_ = create_subscription<test_sharing_data::msg::TestData1m>("chatter_pod", 10, callback);
  }

private:
  rclcpp::Subscription<test_sharing_data::msg::TestData1m>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
