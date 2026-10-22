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
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(mode_switch_test test/mode_switch_test.cpp)
  target_compile_features(mode_switch_test PUBLIC cxx_std_17)
  target_compile_options(mode_switch_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME mode_switch_test COMMAND mode_switch_test)
  add_test(NAME mode_switch_ros_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/mode_switch_ros_test.py
      $<TARGET_FILE:body_control_node>)
  set_tests_properties(mode_switch_ros_test PROPERTIES TIMEOUT 60)
  add_test(NAME reverse_light_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/reverse_light_test.py
      $<TARGET_FILE:body_control_node>)
  set_tests_properties(reverse_light_test PROPERTIES TIMEOUT 45)
  add_test(NAME turn_light_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/turn_light_test.py
      $<TARGET_FILE:body_control_node>)
  set_tests_properties(turn_light_test PROPERTIES TIMEOUT 45)
  add_test(NAME brake_light_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/brake_light_test.py
      $<TARGET_FILE:body_control_node>)
  set_tests_properties(brake_light_test PROPERTIES TIMEOUT 45)
  add_executable(hazard_light_test test/hazard_light_test.cpp)
  target_compile_features(hazard_light_test PUBLIC cxx_std_17)
  target_compile_options(hazard_light_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME hazard_light_test COMMAND hazard_light_test)
  add_test(NAME hazard_light_ros_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/hazard_light_ros_test.py
      $<TARGET_FILE:body_control_node>)
  set_tests_properties(hazard_light_ros_test PROPERTIES TIMEOUT 75)
  add_test(NAME ads_light_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/ads_light_test.py
      $<TARGET_FILE:body_control_node>)
  set_tests_properties(ads_light_test PROPERTIES TIMEOUT 45)
  add_executable(aux_light_test test/aux_light_test.cpp)
  target_compile_features(aux_light_test PUBLIC cxx_std_17)
  target_compile_options(aux_light_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME aux_light_test COMMAND aux_light_test)
  add_test(NAME aux_light_ros_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/aux_light_ros_test.py
      $<TARGET_FILE:body_control_node>)
  set_tests_properties(aux_light_ros_test PROPERTIES TIMEOUT 45)
  add_executable(fog_light_test test/fog_light_test.cpp)
  target_compile_features(fog_light_test PUBLIC cxx_std_17)
  target_compile_options(fog_light_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME fog_light_test COMMAND fog_light_test)
  add_test(NAME fog_light_ros_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/fog_light_ros_test.py
      $<TARGET_FILE:body_control_node>)
  set_tests_properties(fog_light_ros_test PROPERTIES TIMEOUT 45)
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
  add_executable(unlock_voice_test test/unlock_voice_test.cpp)
  target_compile_features(unlock_voice_test PUBLIC cxx_std_17)
  target_compile_options(unlock_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME unlock_voice_test COMMAND unlock_voice_test)
  add_executable(cargo_completion_voice_test test/cargo_completion_voice_test.cpp)
  target_compile_features(cargo_completion_voice_test PUBLIC cxx_std_17)
  target_compile_options(cargo_completion_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME cargo_completion_voice_test COMMAND cargo_completion_voice_test)
  add_executable(door_open_voice_test test/door_open_voice_test.cpp)
  target_compile_features(door_open_voice_test PUBLIC cxx_std_17)
  target_compile_options(door_open_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME door_open_voice_test COMMAND door_open_voice_test)
  add_executable(start_voice_test test/start_voice_test.cpp)
  target_compile_features(start_voice_test PUBLIC cxx_std_17)
  target_compile_options(start_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME start_voice_test COMMAND start_voice_test)
  add_executable(traffic_wait_voice_test test/traffic_wait_voice_test.cpp)
  target_compile_features(traffic_wait_voice_test PUBLIC cxx_std_17)
  target_compile_options(traffic_wait_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME traffic_wait_voice_test COMMAND traffic_wait_voice_test)
  add_executable(automatic_voice_test test/automatic_voice_test.cpp)
  target_compile_features(automatic_voice_test PUBLIC cxx_std_17)
  target_compile_options(automatic_voice_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME automatic_voice_test COMMAND automatic_voice_test)
endif()

ament_package()
