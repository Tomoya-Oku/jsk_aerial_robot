# 使うCMakeの最小バージョン
cmake_minimum_required(VERSION 3.0.2)

# プロジェクト名=パッケージ名
project(manica)

add_compile_options(-std=c++17)

# 依存パッケージを解決
# 以後 ${catkin_LIBRARIES}, ${catkin_INCLUDE_DIRS} が使える
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  diagnostic_msgs
  geometry_msgs
  message_generation
  pluginlib
  mujoco_ros_control
)

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

# 下流パッケージへ「このパッケージは何を公開するか」を宣言
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manica
  CATKIN_DEPENDS roscpp rospy std_msgs aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib diagnostic_msgs geometry_msgs message_runtime
  DEPENDS system_lib
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
)

# ライブラリ
add_library(manica_controllib
  src/manica.cpp
)

# 依存リンク
target_link_libraries(manica_controllib
  ${catkin_LIBRARIES}
  # ${CERES_LIBRARIES}
)

# C++17など規格指定
set_target_properties(manica_controllib PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
)

# 実行ノードがこのライブラリを使う例
add_executable(manica_node src/manica_node.cpp)
target_link_libraries(manica_node
  manica_controllib
  ${catkin_LIBRARIES}
)

# install（他パッケージから使えるように）
install(TARGETS manica_controllib manica_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)