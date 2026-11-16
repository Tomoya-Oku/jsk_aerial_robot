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
find_package(gazebo REQUIRED)
add_service_files(FILES ShapeFeasibility.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs visualization_msgs
  geometry_msgs nav_msgs std_srvs controller_manager_msgs aerial_robot_model
  aerial_robot_msgs dragon message_runtime pluginlib)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_library(wall_contact SHARED src/wall_contact.cpp)
target_link_libraries(wall_contact ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_executable(shape_feasibility_node src/shape_feasibility_node.cpp)
add_dependencies(shape_feasibility_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(shape_feasibility_node ${catkin_LIBRARIES})
catkin_install_python(PROGRAMS scripts/mkii/control.py scripts/mkii/panel.py scripts/mkii/direct.py
  scripts/control/wall.py scripts/control/control_joint_angle.py scripts/control/shape_input.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS wall_contact LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS shape_feasibility_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch scripts urdf meshes docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_contact.py)
  catkin_add_nosetests(test/test_output.py)
  catkin_add_nosetests(test/test_runtime.py)
  catkin_add_nosetests(scripts/haptics/test_haptics.py)
  add_test(NAME mkii_current COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/mkii/current.py)
  add_test(NAME mkii_model COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/mkii/model.py)
endif()
