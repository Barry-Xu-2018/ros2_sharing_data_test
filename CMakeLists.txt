cmake_minimum_required(VERSION 3.8)
project(test_sharing_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

# Create message
set(msg_files
  "msg/TestData1m.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

add_executable(pod_data_listener src/pod_data_listener.cpp)
ament_target_dependencies(pod_data_listener rclcpp)

add_executable(pod_data_talker src/pod_data_talker.cpp)
ament_target_dependencies(pod_data_talker rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pod_data_listener ${cpp_typesupport_target})
target_link_libraries(pod_data_talker ${cpp_typesupport_target})

install(TARGETS
  pod_data_listener
  pod_data_talker
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
