cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_capabilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(capabilities2_events REQUIRED)
find_package(capabilities2_msgs REQUIRED)
find_package(capabilities2_runner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(mobile_manipulator_msgs REQUIRED)
find_package(mobile_manipulator_validator REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/mobile_manipulator_runners.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  capabilities2_events
  capabilities2_msgs
  capabilities2_runner
  geometry_msgs
  grasping_msgs
  mobile_manipulator_msgs
  nav2_msgs
  pluginlib
  rclcpp
  rclcpp_action
)

pluginlib_export_plugin_description_file(capabilities2_runner plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY interfaces
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY providers
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_costmap_validator
    test/test_costmap_validator.cpp
  )
  if(TARGET test_costmap_validator)
    ament_target_dependencies(test_costmap_validator
      geometry_msgs
      mobile_manipulator_validator
      nav2_msgs
    )
    target_include_directories(test_costmap_validator PUBLIC include)
  endif()
endif()

ament_package()