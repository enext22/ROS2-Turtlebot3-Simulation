cmake_minimum_required(VERSION 3.8)
project(obs_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(collision_handler src/collision_handler.cpp)
add_executable(talker src/publish_detection.cpp)
add_executable(listener src/scan_subscriber.cpp)
add_executable(controller src/action_subscriber.cpp)

ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs geometry_msgs)
ament_target_dependencies(controller rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)


target_include_directories(collision_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(collision_handler PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS collision_handler
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS talker
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS listener
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
