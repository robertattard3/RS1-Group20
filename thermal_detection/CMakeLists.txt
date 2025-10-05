cmake_minimum_required(VERSION 3.8)
project(thermal_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install resources (worlds, URDF, etc.)
install(
  DIRECTORY config launch models urdf urdf_drone worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python node(s) so `ros2 launch` can execute them
install(
  PROGRAMS
    scripts/thermal_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

