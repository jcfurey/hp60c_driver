cmake_minimum_required(VERSION 3.16)
project(hp60c_driver CXX)

# Optional CUDA backend for depth unpacking + registration (the costliest
# per-frame work on the CPU). Used automatically where CUDA exists (Jetson);
# elsewhere, e.g. the x86 CI/Docker images, the CPU path is built alone.
option(HP60C_WITH_CUDA "Build the CUDA backend if a CUDA compiler is found" ON)
if(HP60C_WITH_CUDA)
  if(NOT CMAKE_CUDA_COMPILER AND EXISTS /usr/local/cuda/bin/nvcc)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)   # JetPack: not on PATH
  endif()
  include(CheckLanguage)
  check_language(CUDA)
endif()
if(HP60C_WITH_CUDA AND CMAKE_CUDA_COMPILER)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 87)   # Jetson Orin; override for other GPUs
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(HP60C_CUDA ON)
  message(STATUS "hp60c_driver: CUDA backend enabled (${CMAKE_CUDA_COMPILER})")
else()
  set(HP60C_CUDA OFF)
  message(STATUS "hp60c_driver: CUDA backend disabled; CPU path only")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # C++ only: nvcc's generated host code trips -Wpedantic (line directives).
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall> $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED IMPORTED_TARGET libturbojpeg)

# Format parsing, depth filtering, camera-control logic and software auto
# exposure: no ROS, so their unit tests need neither a camera nor rclcpp.
add_library(hp60c_frame STATIC
  src/frame.cpp
  src/depth_filter.cpp
  src/camera_controls.cpp
  src/auto_exposure.cpp)
set_target_properties(hp60c_frame PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hp60c_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(HP60C_CUDA)
  add_library(hp60c_cuda STATIC src/cuda_registrar.cu)
  set_target_properties(hp60c_cuda PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(hp60c_cuda PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(hp60c_cuda PUBLIC CUDA::cudart)
endif()

# Imported targets only: ament_target_dependencies() is gone in Lyrical.
add_library(hp60c_component SHARED
  src/hp60c_node.cpp
  src/v4l2_capture.cpp)
target_link_libraries(hp60c_component
  hp60c_frame
  rclcpp::rclcpp
  rclcpp_components::component
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  PkgConfig::TURBOJPEG)
if(HP60C_CUDA)
  target_link_libraries(hp60c_component hp60c_cuda)
  target_compile_definitions(hp60c_component PRIVATE HP60C_WITH_CUDA)
endif()
rclcpp_components_register_node(hp60c_component
  PLUGIN "hp60c_driver::Hp60cNode"
  EXECUTABLE hp60c_node)

install(TARGETS hp60c_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame test/test_frame.cpp)
  target_link_libraries(test_frame hp60c_frame)
  ament_add_gtest(test_depth_filter test/test_depth_filter.cpp)
  target_link_libraries(test_depth_filter hp60c_frame)
  ament_add_gtest(test_camera_controls test/test_camera_controls.cpp)
  target_link_libraries(test_camera_controls hp60c_frame)
  ament_add_gtest(test_auto_exposure test/test_auto_exposure.cpp)
  target_link_libraries(test_auto_exposure hp60c_frame)
  # The node itself, in-process and without a camera: parameters, their
  # validation, diagnostics and a clean shutdown.
  ament_add_gtest(test_node test/test_node.cpp TIMEOUT 120)
  target_link_libraries(test_node hp60c_component)
  if(HP60C_CUDA)
    ament_add_gtest(test_cuda_registrar test/test_cuda_registrar.cpp)
    target_link_libraries(test_cuda_registrar hp60c_frame hp60c_cuda)
  endif()

  find_package(ament_lint_auto REQUIRED)
  # uncrustify's bundled config differs between Jazzy, Kilted and Lyrical, so a
  # file formatted on one fails on another. Skip it; cpplint still checks style.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
