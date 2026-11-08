cmake_minimum_required(VERSION 3.16)
project(starai_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# Protocol + serial bus: no ROS dependencies, unit-testable on its own.
add_library(fashionstar_bus STATIC
  src/fashionstar_protocol.cpp
  src/fashionstar_bus.cpp
)
set_target_properties(fashionstar_bus PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fashionstar_bus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(starai_hardware SHARED src/fashionstar_system.cpp)
target_link_libraries(starai_hardware PUBLIC fashionstar_bus)
target_include_directories(starai_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(starai_hardware PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface starai_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS fashionstar_bus starai_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS
  scripts/fake_fashionstar_bus.py
  scripts/go_to_pose.py
  scripts/teach_mode
  scripts/track_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol fashionstar_bus)
  ament_add_gtest(test_servo_goal test/test_servo_goal.cpp)
  target_link_libraries(test_servo_goal fashionstar_bus)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_go_to_pose test/test_go_to_pose.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
