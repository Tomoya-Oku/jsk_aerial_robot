cmake_minimum_required(VERSION 3.0.2)
project(dracomancer_mkii_ros)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs std_srvs controller_manager_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/control.py scripts/panel.py scripts/direct.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config urdf meshes docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_test(NAME mkii_current COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/current.py)
  add_test(NAME mkii_model COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/model.py)
endif()
