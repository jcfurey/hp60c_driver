cmake_minimum_required(VERSION 3.16)
project(hp60c_driver CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED IMPORTED_TARGET libturbojpeg)

# Format parsing: no ROS, so the unit tests need neither a camera nor rclcpp.
add_library(hp60c_frame STATIC src/frame.cpp)
set_target_properties(hp60c_frame PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hp60c_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Imported targets only: ament_target_dependencies() is gone in Lyrical.
add_library(hp60c_component SHARED
  src/hp60c_node.cpp
  src/v4l2_capture.cpp)
target_link_libraries(hp60c_component
  hp60c_frame
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  PkgConfig::TURBOJPEG)
rclcpp_components_register_node(hp60c_component
  PLUGIN "hp60c_driver::Hp60cNode"
  EXECUTABLE hp60c_node)

install(TARGETS hp60c_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame test/test_frame.cpp)
  target_link_libraries(test_frame hp60c_frame)

  find_package(ament_lint_auto REQUIRED)
  # uncrustify's bundled config differs between Jazzy, Kilted and Lyrical, so a
  # file formatted on one fails on another. Skip it; cpplint still checks style.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
