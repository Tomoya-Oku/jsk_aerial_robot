cmake_minimum_required(VERSION 3.0.2)
project(dracomancer)
add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs visualization_msgs geometry_msgs nav_msgs
  std_srvs controller_manager_msgs aerial_robot_model aerial_robot_msgs
  dragon message_generation pluginlib
)
catkin_python_setup()
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(gazebo REQUIRED)
add_service_files(FILES ShapeFeasibility.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs visualization_msgs
  geometry_msgs nav_msgs std_srvs controller_manager_msgs aerial_robot_model
  aerial_robot_msgs dragon message_runtime pluginlib)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_library(wall_contact SHARED src/simulation/wall_contact.cpp)
target_link_libraries(wall_contact ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_executable(shape_feasibility_node
  src/feasibility/node.cpp
  src/feasibility/server.cpp
  src/feasibility/prediction.cpp
  src/feasibility/allocation.cpp
  src/feasibility/feedback.cpp
  src/feasibility/wrench.cpp)
# DRAGON's public model header includes nlopt.hpp; catkin does not export its path.
target_include_directories(shape_feasibility_node PRIVATE ${NLOPT_INCLUDE_DIRS})
add_dependencies(shape_feasibility_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(shape_feasibility_node ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})
catkin_install_python(PROGRAMS
  scripts/control/control_joint_angle.py scripts/control/control_position.py scripts/control/feedback.py
  scripts/control/shape_input.py scripts/control/wall.py
  scripts/device_io/calibrate_joystick.py scripts/device_io/joystick.py
  scripts/mkii/control.py scripts/mkii/direct.py scripts/mkii/observe.py scripts/mkii/panel.py
  scripts/safety/fc_min_lowpass.py scripts/safety/volume_radius_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS wall_contact LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS shape_feasibility_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch urdf meshes docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(DIRECTORY figures/scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/figures USE_SOURCE_PERMISSIONS)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
