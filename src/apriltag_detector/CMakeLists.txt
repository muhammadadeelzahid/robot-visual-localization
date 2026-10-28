cmake_minimum_required(VERSION 3.8)
project(apriltag_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(apriltag REQUIRED)
find_package(OpenCV REQUIRED)

# Detection library.
add_library(apriltag_detector_lib src/apriltag_detector.cpp)
target_include_directories(apriltag_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(apriltag_detector_lib apriltag::apriltag ${OpenCV_LIBS})

# ROS node.
add_executable(detector_node src/detector_node.cpp)
target_link_libraries(detector_node apriltag_detector_lib ${OpenCV_LIBS})
ament_target_dependencies(detector_node
  rclcpp sensor_msgs geometry_msgs apriltag_msgs cv_bridge image_transport)

install(TARGETS apriltag_detector_lib
  EXPORT export_apriltag_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detection test/test_detection.cpp)
  target_link_libraries(test_detection apriltag_detector_lib ${OpenCV_LIBS})
endif()

ament_export_include_directories(include)
ament_export_libraries(apriltag_detector_lib)
ament_export_dependencies(apriltag OpenCV)
ament_package()
