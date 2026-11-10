cmake_minimum_required(VERSION 3.5)
project(rc_agent)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json QUIET)

find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

# Copy config from source to build
configure_file(
        robot.cfg.in
        robot.cfg
        )

add_subdirectory(system)
add_subdirectory(network)
add_subdirectory(config)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/network/protobuf/generated
  #${nav2_msgs_INCLUDE_DIRS}
  #${rclcpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CONFIG_PROCESSOR
    config/configprocessor.h
    config/configprocessor.cpp
   )

set(VIDEO_SOURCES
    modules/video/stream_pipeline_receiver.h
    modules/video/stream_pipeline_receiver.cpp
)

set(BATTERY_SOURCES
    modules/battery/battery_state_adapter.h
    modules/battery/battery_state_handler.h
    modules/battery/battery_state_handler.cpp
    modules/battery/battery_state_node.h
    modules/battery/battery_state_node.cpp
)

set(NAVIGATOR_SOURCES
    modules/nav2/nav_command.h
    modules/nav2/nav_command.cpp
    modules/nav2/robot_navigator_handler.cpp
    modules/nav2/robot_navigator_handler.h
    modules/nav2/robot_navigator.h
    modules/nav2/robot_navigator.cpp
)

set(BASECONTROL_SOURCES
    modules/basecontrol/basecontrol.h
    modules/basecontrol/basecontrol.cpp
)

set(MAP_SOURCES
    modules/map/map_node.h
    modules/map/map_node.cpp
)

set(MODULES
    ${VIDEO_SOURCES}
    ${BATTERY_SOURCES}
    ${MAP_SOURCES}
    ${NAVIGATOR_SOURCES}
    ${BASECONTROL_SOURCES}
)

add_executable(${PROJECT_NAME}
  main.cpp
  ${CONFIG_PROCESSOR}
  ${MODULES}
)

if(BUILD_TESTING)
  ament_add_gtest(map_endpoint_binding_test
    test/unit_tests/map_endpoint_binding_test.cpp
    modules/map/map_node.cpp
  )
  if(TARGET map_endpoint_binding_test)
    target_include_directories(map_endpoint_binding_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    ament_target_dependencies(map_endpoint_binding_test rclcpp nav_msgs std_msgs tf2 tf2_ros geometry_msgs)
    target_link_libraries(map_endpoint_binding_test RobotProtos)
  endif()

  ament_add_gtest(endpoint_binding_test
    test/unit_tests/endpoint_binding_test.cpp
    modules/basecontrol/basecontrol.cpp
  )
  if(TARGET endpoint_binding_test)
    target_include_directories(endpoint_binding_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    ament_target_dependencies(endpoint_binding_test rclcpp geometry_msgs nav_msgs)
    target_link_libraries(endpoint_binding_test RobotProtos)
  endif()

  ament_add_gtest(battery_freshness_test
    test/unit_tests/battery_freshness_test.cpp
    modules/battery/battery_state_node.cpp
    modules/battery/battery_state_handler.cpp
  )
  if(TARGET battery_freshness_test)
    target_include_directories(battery_freshness_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    ament_target_dependencies(battery_freshness_test rclcpp sensor_msgs)
    target_link_libraries(battery_freshness_test RobotProtos)
  endif()

  ament_add_gtest(video_pipeline_snapshot_test
    test/unit_tests/video_pipeline_snapshot_test.cpp
    modules/video/stream_pipeline_receiver.cpp
  )
  if(TARGET video_pipeline_snapshot_test)
    target_include_directories(video_pipeline_snapshot_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    ament_target_dependencies(video_pipeline_snapshot_test rclcpp std_msgs)
    target_link_libraries(video_pipeline_snapshot_test RobotProtos)
  endif()
endif()

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
  action_msgs
  lifecycle_msgs
)

if(nlohmann_json_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_NLOHMANN_JSON)
  target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)
endif()

target_link_libraries(${PROJECT_NAME}
  NetworkController
  ${Boost_LIBRARIES}
)

if(libСonfig_LIB)
    target_link_libraries(${PROJECT_NAME} config++)
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
        DESTINATION share/${PROJECT_NAME})

ament_package()
