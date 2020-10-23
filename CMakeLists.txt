cmake_minimum_required(VERSION 3.5)

project(rmf_web_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_task_ros2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
include(GNUInstallDirs)

#===============================================================================

find_package(libmongocxx REQUIRED)
find_package(libbsoncxx REQUIRED)
include_directories(${LIBMONGOCXX_INCLUDE_DIR})
include_directories(${LIBBSONCXX_INCLUDE_DIR})
include_directories("/usr/local/include/mongocxx/v_noabi")
include_directories("/usr/local/include/bsoncxx/v_noabi")
include_directories("/usr/local/include/libmongoc-1.0")
include_directories("/usr/local/include/libbson-1.0")
include_directories("/usr/local/lib")

add_executable(dispatcher_db_client
  src/main.cpp
)

target_link_libraries(dispatcher_db_client 
  PRIVATE
    ${LIBMONGOCXX_LIBRARIES}
    ${LIBBSONCXX_LIBRARIES}
    ${rclcpp_LIBRARIES}
    rmf_traffic::rmf_traffic
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_task_ros2::rmf_task_ros2
    ${rmf_task_msgs_LIBRARIES}
)

target_include_directories(dispatcher_db_client
  PRIVATE
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rmf_task_ros2_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

#===============================================================================
# install(
#   DIRECTORY include/
#   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
# )

install(
  TARGETS dispatcher_db_client
  RUNTIME DESTINATION lib/rmf_web_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
