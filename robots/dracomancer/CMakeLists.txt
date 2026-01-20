# 使うCMakeの最小バージョン
cmake_minimum_required(VERSION 3.0.2)

# プロジェクト名=パッケージ名
project(dracomancer)

add_compile_options(-std=c++17)

# 依存パッケージを解決
# 以後 ${catkin_LIBRARIES}, ${catkin_INCLUDE_DIRS} が使える
find_package(catkin REQUIRED COMPONENTS
  # roscpp
  # rospy
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
  LIBRARIES dracomancer
  CATKIN_DEPENDS roscpp std_msgs aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib diagnostic_msgs geometry_msgs message_runtime
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

# 依存リンク
add_library(dracomancer_controllib src/control/dracomancer_controller.cpp src/control/dracomancer_optimizer.cpp)
target_link_libraries(dracomancer_controllib ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

add_library(dracomancer_model src/model/dracomancer_model.cpp)
target_link_libraries(dracomancer_model ${catkin_LIBRARIES})

# install（他パッケージから使えるように）
install(TARGETS dracomancer_controllib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY config plugins launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)