cmake_minimum_required(VERSION 3.8)
project(body_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(body_control_node
  src/body_control_node.cpp
)

target_compile_features(body_control_node PUBLIC cxx_std_17)

ament_target_dependencies(body_control_node
  rclcpp
  std_msgs
)

install(TARGETS body_control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(voice_scene_test test/voice_scene_test.cpp)
  target_compile_features(voice_scene_test PUBLIC cxx_std_17)
  target_compile_options(voice_scene_test PRIVATE -UNDEBUG)
  add_test(NAME voice_scene_test COMMAND voice_scene_test)
  add_executable(voice_scheduler_test test/voice_scheduler_test.cpp)
  target_compile_features(voice_scheduler_test PUBLIC cxx_std_17)
  target_compile_options(voice_scheduler_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME voice_scheduler_test COMMAND voice_scheduler_test)
  add_executable(remote_voice_test test/remote_voice_test.cpp)
  target_compile_features(remote_voice_test PUBLIC cxx_std_17)
  target_compile_options(remote_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME remote_voice_test COMMAND remote_voice_test)
  add_executable(low_battery_voice_test test/low_battery_voice_test.cpp)
  target_compile_features(low_battery_voice_test PUBLIC cxx_std_17)
  target_compile_options(low_battery_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME low_battery_voice_test COMMAND low_battery_voice_test)
  add_executable(cargo_voice_test test/cargo_voice_test.cpp)
  target_compile_features(cargo_voice_test PUBLIC cxx_std_17)
  target_compile_options(cargo_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME cargo_voice_test COMMAND cargo_voice_test)
endif()

ament_package()
