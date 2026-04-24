cmake_minimum_required(VERSION 3.5)
project(spiderfish_gazebo)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(spiderfish_interfaces REQUIRED)

include_directories(include)
include_directories(gazebo_utils)
include_directories(math_utils)
include_directories(${OpenCV_INCLUDE_DIRS})


# Ground Truth Sensor Plugin
add_library(ground_truth_sensor SHARED src/ground_truth_sensor.cpp)
target_include_directories(ground_truth_sensor PUBLIC include)
ament_target_dependencies(ground_truth_sensor "ros_gz_sim")
ament_export_libraries(ground_truth_sensor)
target_link_libraries(ground_truth_sensor ${geometry_msgs_LIBRARIES} gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
install(TARGETS ground_truth_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Thruster Driver Plugin
add_library(thruster_driver_plugin SHARED src/thruster_driver_plugin.cpp)
target_include_directories(thruster_driver_plugin PUBLIC include)
ament_target_dependencies(thruster_driver_plugin "ros_gz_sim")
ament_export_libraries(thruster_driver_plugin)
target_link_libraries(thruster_driver_plugin gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
install(TARGETS thruster_driver_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Actuators Command Simulation Executable
add_executable(actuators_command_simulation src/actuators_command_simulation.cpp)
target_include_directories(actuators_command_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(actuators_command_simulation
  "rclcpp"
  "rclcpp_components"
  "spiderfish_interfaces"
  "ros_gz_interfaces"
  "ament_index_cpp"
)
install(TARGETS actuators_command_simulation DESTINATION lib/${PROJECT_NAME})


# Hydrodynamics plugin
add_library(hydrodynamics_plugin SHARED
  src/hydrodynamics_plugin.cpp
  gazebo_utils/src/gazebo_utils.cpp
  math_utils/src/math_utils.cpp
)
target_include_directories(hydrodynamics_plugin PUBLIC include gazebo_utils math_utils)
ament_target_dependencies(hydrodynamics_plugin "ros_gz_sim")
ament_export_libraries(hydrodynamics_plugin)
target_link_libraries(hydrodynamics_plugin gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
install(TARGETS hydrodynamics_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Underwater Camera Executable - DEPRECATED (FROM GAZEBO CLASSIC)
# add_executable(underwater_camera src/underwater_camera.cpp)
# target_include_directories(underwater_camera PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(underwater_camera
#   "image_transport"
#   "sensor_msgs"
#   "message_filters"
#   "rclcpp"
#   "OpenCV"
#   "cv_bridge"
#   "spiderfish_interfaces"
# )
# install(TARGETS underwater_camera DESTINATION lib/${PROJECT_NAME})


# Bounding Box Plugin - DEPRECATED (FROM GAZEBO CLASSIC)
# add_library(bounding_box_plugin SHARED src/bounding_box_plugin.cpp)
# target_include_directories(bounding_box_plugin PUBLIC include)
# ament_target_dependencies(bounding_box_plugin "spiderfish_interfaces")
# ament_export_libraries(bounding_box_plugin)
# target_link_libraries(bounding_box_plugin gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
# install(TARGETS bounding_box_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


# Camera Orbit Plugin - DEPRECATED (FROM GAZEBO CLASSIC)
# add_library(camera_orbit_plugin SHARED src/camera_orbit_plugin.cpp)
# target_include_directories(camera_orbit_plugin PUBLIC include)
# ament_target_dependencies(camera_orbit_plugin "spiderfish_interfaces")
# ament_export_libraries(camera_orbit_plugin)
# target_link_libraries(camera_orbit_plugin gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
# install(TARGETS camera_orbit_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


# Random Camera Plugin - DEPRECATED (FROM GAZEBO CLASSIC)
# add_library(random_camera_plugin SHARED src/random_camera_plugin.cpp)
# target_include_directories(random_camera_plugin PUBLIC include)
# ament_target_dependencies(random_camera_plugin "spiderfish_interfaces")
# ament_export_libraries(random_camera_plugin)
# target_link_libraries(random_camera_plugin gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
# install(TARGETS random_camera_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


# Smooth Camera Plugin - DEPRECATED (FROM GAZEBO CLASSIC)
# add_library(smooth_camera_plugin SHARED src/smooth_camera_plugin.cpp)
# target_include_directories(smooth_camera_plugin PUBLIC include)
# ament_target_dependencies(smooth_camera_plugin "spiderfish_interfaces")
# ament_export_libraries(smooth_camera_plugin)
# target_link_libraries(smooth_camera_plugin gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
# install(TARGETS smooth_camera_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


# Spiderfish Gazebo Library
# add_library(spiderfish_gazebo SHARED src/underwater_camera.cpp)
# target_compile_definitions(spiderfish_gazebo PRIVATE "spiderfish_gazebo_CPP_BUILDING_DLL")
# ament_target_dependencies(spiderfish_gazebo
#   "rclcpp"
#   "rclcpp_components"
#   "image_transport"
#   "sensor_msgs"
#   "message_filters"
#   "spiderfish_interfaces"
#   "geometry_msgs"
# )
# install(TARGETS spiderfish_gazebo
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


# Build everything else
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/gazebo)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/gazebo)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  spiderfish_gazebo/bounding_box_image_saver.py
  spiderfish_gazebo/train_yolo.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(ament_cmake_python)


if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_gazebo_launch.py
    TARGET test_gazebo_launch_default_world
    ARGS "headless:=true"
  )
    
  add_launch_test(
    test/test_gazebo_launch.py
    TARGET test_gazebo_launch_cube_world
    ARGS "headless:=true" "world:=cube.world"
  )
endif()

ament_package()