cmake_minimum_required(VERSION 3.8)
project(search_and_rescue)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# add any other packages as we go
find_package(ament_cmake REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)  
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(navigation src/mission_main.cpp src/mission.cpp)
target_link_libraries(navigation)
ament_target_dependencies(navigation rclcpp geometry_msgs std_msgs nav_msgs nav2_msgs rclcpp_action)

install(TARGETS
  navigation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()