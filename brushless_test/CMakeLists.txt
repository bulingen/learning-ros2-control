cmake_minimum_required(VERSION 3.8)
project(brushless_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(test_motors src/test_motors.cpp)
ament_target_dependencies(test_motors rclcpp)

target_include_directories(test_motors PRIVATE /usr/include)  # pigpio.h lives here
target_link_libraries(test_motors pigpio pthread rt) # pthread and rt required by pigpio internally

install(TARGETS
  test_motors
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
