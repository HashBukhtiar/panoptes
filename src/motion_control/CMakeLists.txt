cmake_minimum_required(VERSION 3.20)
project(motion_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(kinematics REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_executable(motion_control_node src/motion_control_node.cpp)
target_include_directories(motion_control_node PRIVATE include)

target_link_libraries(motion_control_node PRIVATE
  rclcpp::rclcpp
  ${interfaces_TARGETS}
  kinematics::kinematics
  sensor_msgs::sensor_msgs
)
install(TARGETS motion_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # pure math, no ROS needed
    ament_add_gtest(test_trajectory
        test/test_trajectory.cpp
    )
    target_include_directories(test_trajectory PRIVATE include)

    # node communication, uses the stubbed kinematics
    # ament_add_gtest(test_motion_control_pubsub
    #     test/test_motion_control_pubsub.cpp
    #     test/stubs/stub_kinematics.cpp
    # )
    # target_include_directories(test_motion_control_pubsub PRIVATE include)
    # target_link_libraries(test_motion_control_pubsub
    #     rclcpp::rclcpp
    #     ${sensor_msgs_TARGETS}
    #     ${interfaces_TARGETS}
    # )
endif()

ament_package()
