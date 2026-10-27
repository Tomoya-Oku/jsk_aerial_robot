# 使うCMakeの最小バージョン
cmake_minimum_required(VERSION 3.0.2)

# プロジェクト名=パッケージ名
project(dracomancer)

add_compile_options(-std=c++17)

# 依存パッケージを解決
# 以後 ${catkin_LIBRARIES}, ${catkin_INCLUDE_DIRS} が使える
find_package(catkin REQUIRED COMPONENTS
  roscpp
  # rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  diagnostic_msgs
  geometry_msgs
  dragon
  message_generation
  pluginlib
  mujoco_ros_control
)

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(gazebo REQUIRED)

# メッセージ・サービス定義
add_message_files(
  FILES
  MappingState.msg
)
add_service_files(
  FILES
  ShapeFeasibility.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 下流パッケージへ「このパッケージは何を公開するか」を宣言
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dracomancer
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs visualization_msgs aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib diagnostic_msgs geometry_msgs dragon message_runtime
)

###########
## Build ##
###########

# 公開ヘッダは include/プロジェクト名/ 以下へ
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(wall_contact SHARED src/wall_contact.cpp)
target_link_libraries(wall_contact ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
install(TARGETS wall_contact LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

catkin_install_python(PROGRAMS scripts/control/wall.py scripts/control/contact_haptics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 依存リンク
add_library(dracomancer_controllib src/control/dracomancer_controller.cpp src/control/dracomancer_optimizer.cpp)
target_link_libraries(dracomancer_controllib ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(dracomancer_model src/model/dracomancer_model.cpp)
target_link_libraries(dracomancer_model ${catkin_LIBRARIES})

# 候補姿勢の force/torque volume 半径を予測するサービスノード
add_executable(shape_feasibility_node src/shape_feasibility_node.cpp)
add_dependencies(shape_feasibility_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(shape_feasibility_node ${catkin_LIBRARIES})

# install（他パッケージから使えるように）
install(TARGETS dracomancer_controllib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS shape_feasibility_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config plugins launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_contact.py)
  catkin_add_nosetests(test/test_haptics.py)
endif()
