// Copyright 2023 Nick Morales.
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
//
// # CHANGES:
//
// 2024-11-09 , Tommy Chang
//     - Use modern ROS2 syntax
//     - Add comments
//     - Remove unnecessary lamda expression

/// @file A dummy ROS 2 node that provides a service for the test node
/// to check.

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

auto Logger = rclcpp::get_logger (""); // create an initial Logger

// callback function
void callback (std_srvs::srv::Empty::Request::SharedPtr  request,
               std_srvs::srv::Empty::Response::SharedPtr response)
{
  RCLCPP_INFO (Logger, "calling callback");
}


int main(int argc, char * argv[])
{
  // 1.) Initialize ROS 2 C++ client library
  RCLCPP_INFO (Logger, "initialize ROS2");
  rclcpp::init(argc, argv);

  // 2.) Create a service server (with a specific service name)
  RCLCPP_INFO (Logger, "Create service_server_node");
  auto node = rclcpp::Node::make_shared ("service_server_node");
  auto service = node->create_service <std_srvs::srv::Empty> ("myServiceName", &callback);

  // change the named Logger to match the name of the running node so that it
  // will appears in rqt_console
  Logger = node->get_logger();
  
  // 3.) Start the server
  RCLCPP_INFO (Logger, "Start server");
  rclcpp::spin (node);          // blocking call

  // 4.) Shutdown ROS 2
  RCLCPP_INFO (Logger, "Shutdown ROS2");
  rclcpp::shutdown();
  return 0;
}
