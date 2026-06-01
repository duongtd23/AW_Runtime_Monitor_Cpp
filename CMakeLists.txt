cmake_minimum_required(VERSION 3.8)
project(aw_runtime_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
find_package(aw_monitor REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(autoware_route_handler REQUIRED)

include_directories(include
  /usr/local/include
)
link_directories(
  /usr/local/lib
)

add_executable(aw_rt_monitor 
  src/planning/planning_shield.cpp
  src/planning/trajectory_revision.cpp
  src/perception/perception_shield.cpp
  src/control/control_command.cpp
  src/recorder/aw_recorder.cpp
  src/aw_rt_monitor.cpp
)

ament_target_dependencies(aw_rt_monitor
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_control_msgs
  autoware_adapi_v1_msgs
  autoware_vehicle_msgs
  tier4_perception_msgs
  aw_monitor
  cv_bridge
  image_transport
  OpenCV
  autoware_route_handler
  autoware_map_msgs
)

target_link_libraries(aw_rt_monitor
  spot
  bddx
  Eigen3::Eigen
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS
  aw_rt_monitor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # fqltl — grounding engine tests
  ament_add_gtest(test_fqltl_grounder
    test/fqltl/test_grounder.cpp
  )
  target_include_directories(test_fqltl_grounder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  # fqltl — evaluator tests
  ament_add_gtest(test_fqltl_evaluator
    test/fqltl/test_evaluator.cpp
  )
  target_include_directories(test_fqltl_evaluator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/local/include
  )
  target_link_libraries(test_fqltl_evaluator
    spot
    bddx
    Eigen3::Eigen
  )
  ament_target_dependencies(test_fqltl_evaluator
    rclcpp
    geometry_msgs
    autoware_perception_msgs
    autoware_planning_msgs
    autoware_vehicle_msgs
    tf2
    tf2_geometry_msgs
  )
endif()

ament_package()
