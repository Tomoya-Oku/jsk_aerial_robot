#pragma once

#include <aerial_robot_model/model/aerial_robot_model.h>
#include <dracomancer/ShapeFeasibility.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace Dragon
{
class FullVectoringRobotModel;
}

namespace dracomancer
{
class ShapeFeasibilityServer
{
public:
  ShapeFeasibilityServer(ros::NodeHandle nh, ros::NodeHandle nhp);

private:
  bool checkShape(ShapeFeasibility::Request& req, ShapeFeasibility::Response& res);

  bool evaluateModelFc(double& force, double& torque);
  bool evaluateOptimizedGimbalFc(double& force, double& torque);
  bool evaluateControllerFeedbackFc(double& force, double& torque);
  bool evaluateAllocationFc(double& force, double& torque);
  bool evaluateFixedWrench(aerial_robot_model::RobotModel& model, double& force, double& torque);

  boost::shared_ptr<aerial_robot_model::RobotModel> preparePlanModel(
    Dragon::FullVectoringRobotModel& dragon_model, const KDL::JntArray& evaluated_joint,
    const Eigen::Matrix3d& cog_rot);
  std::vector<Eigen::Matrix3d> linksRotationFromCog(aerial_robot_model::RobotModel& model);
  void setJointPosition(const std::map<std::string, unsigned int>& joint_index_map,
                        KDL::JntArray& joint_positions, const std::string& joint_name,
                        double value) const;
  static KDL::Rotation matrixToKdlRotation(const Eigen::Matrix3d& rotation);

  Eigen::VectorXd allocationTargetAcc(const Dragon::FullVectoringRobotModel& dragon_model) const;
  bool staticHoverAllocation(const Dragon::FullVectoringRobotModel& dragon_model,
                             aerial_robot_model::RobotModel& plan_model,
                             const Eigen::VectorXd& target_acc,
                             const std::vector<int>& roll_locked_gimbal,
                             const std::vector<double>& gimbal_nominal_angles,
                             const std::vector<Eigen::Matrix3d>& links_rotation_from_cog,
                             KDL::JntArray& gimbal_processed_joint,
                             std::vector<double>& thrust_forces,
                             std::vector<double>& gimbal_angles,
                             Eigen::VectorXd& vectoring_forces);
  void applyHoverVectoringFallback(const Dragon::FullVectoringRobotModel& dragon_model,
                                   int hover_index, double& force_component) const;

  bool latestGimbalState(const Dragon::FullVectoringRobotModel& dragon_model,
                         KDL::JntArray& feedback_joint, std::vector<double>& rolls);
  std::pair<bool, Eigen::Matrix3d> latestBaselinkRotation();
  static Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw);
  void gimbalCb(const sensor_msgs::JointStateConstPtr& msg);
  void baselinkRpyCb(const geometry_msgs::Vector3StampedConstPtr& msg);
  void baselinkOdomCb(const nav_msgs::OdometryConstPtr& msg);

  ros::NodeHandle nh_, nhp_;
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  ros::ServiceServer server_;
  ros::Subscriber gimbal_sub_, baselink_odom_sub_, baselink_rpy_sub_;
  std::string prediction_mode_;
  int force_dimensions_;
  Eigen::VectorXd external_wrench_ = Eigen::VectorXd::Zero(6);
  bool external_in_link1_ = false;
  std::vector<std::string> contact_links_;
  std::vector<double> contact_points_, contact_wrenches_;
  std::vector<bool> contact_local_;
  std::vector<double> evaluated_gimbals_, evaluated_rotation_;
  std::vector<double> fc_to_cog_;
  std::string gimbal_feedback_topic_, baselink_odom_topic_, baselink_rpy_topic_;
  bool use_gimbal_feedback_, use_baselink_odom_feedback_, use_baselink_rpy_feedback_;
  double feedback_timeout_;
  int allocation_refine_max_iteration_;
  double allocation_refine_threshold_;
  bool allocation_start_rp_integration_;
  bool use_allocation_target_acc_param_ = false;
  Eigen::VectorXd allocation_target_acc_ = Eigen::VectorXd::Zero(6);
  std::mutex feedback_mutex_;
  bool have_gimbal_feedback_ = false;
  bool have_baselink_odom_feedback_ = false;
  bool have_baselink_rpy_feedback_ = false;
  ros::Time latest_gimbal_stamp_, latest_baselink_odom_stamp_, latest_baselink_rpy_stamp_;
  std::map<std::string, double> latest_gimbal_positions_;
  Eigen::Matrix3d latest_baselink_rot_ = Eigen::Matrix3d::Identity();
  double latest_baselink_rpy_[3] = {0.0, 0.0, 0.0};
};
}  // namespace dracomancer
