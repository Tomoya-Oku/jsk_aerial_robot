#include "server.h"

#include <dragon/model/full_vectoring_robot_model.h>
#include <Eigen/Geometry>

namespace dracomancer
{
bool ShapeFeasibilityServer::latestGimbalState(
  const Dragon::FullVectoringRobotModel& dragon_model, KDL::JntArray& feedback_joint,
  std::vector<double>& rolls)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!have_gimbal_feedback_ ||
      (ros::Time::now() - latest_gimbal_stamp_).toSec() > feedback_timeout_)
    {
      return false;
    }

  rolls.clear();
  const auto& joint_index_map = dragon_model.getJointIndexMap();
  for (int i = 0; i < dragon_model.getRotorNum(); ++i)
    {
      const std::string prefix = "gimbal" + std::to_string(i + 1);
      const std::string roll_name = prefix + "_roll";
      const std::string pitch_name = prefix + "_pitch";
      const auto roll_feedback = latest_gimbal_positions_.find(roll_name);
      const auto pitch_feedback = latest_gimbal_positions_.find(pitch_name);
      const auto roll_index = joint_index_map.find(roll_name);
      const auto pitch_index = joint_index_map.find(pitch_name);
      if (roll_feedback == latest_gimbal_positions_.end() ||
          pitch_feedback == latest_gimbal_positions_.end() ||
          roll_index == joint_index_map.end() || pitch_index == joint_index_map.end())
        {
          return false;
        }
      feedback_joint(roll_index->second) = roll_feedback->second;
      feedback_joint(pitch_index->second) = pitch_feedback->second;
      rolls.push_back(roll_feedback->second);
    }
  return true;
}

std::pair<bool, Eigen::Matrix3d> ShapeFeasibilityServer::latestBaselinkRotation()
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (have_baselink_rpy_feedback_ &&
      (ros::Time::now() - latest_baselink_rpy_stamp_).toSec() <= feedback_timeout_)
    {
      return std::make_pair(true, rpyToMatrix(latest_baselink_rpy_[0],
                                              latest_baselink_rpy_[1],
                                              latest_baselink_rpy_[2]));
    }
  if (have_baselink_odom_feedback_ &&
      (ros::Time::now() - latest_baselink_odom_stamp_).toSec() <= feedback_timeout_)
    {
      return std::make_pair(true, latest_baselink_rot_);
    }
  return std::make_pair(false, Eigen::Matrix3d::Identity());
}

Eigen::Matrix3d ShapeFeasibilityServer::rpyToMatrix(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

void ShapeFeasibilityServer::gimbalCb(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  latest_gimbal_positions_.clear();
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
      latest_gimbal_positions_[msg->name[i]] = msg->position[i];
    }
  latest_gimbal_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  have_gimbal_feedback_ = true;
}

void ShapeFeasibilityServer::baselinkRpyCb(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  latest_baselink_rpy_[0] = msg->vector.x;
  latest_baselink_rpy_[1] = msg->vector.y;
  latest_baselink_rpy_[2] = msg->vector.z;
  latest_baselink_rpy_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  have_baselink_rpy_feedback_ = true;
}

void ShapeFeasibilityServer::baselinkOdomCb(const nav_msgs::OdometryConstPtr& msg)
{
  const auto& q = msg->pose.pose.orientation;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  if (quat.norm() < 1e-9)
    {
      return;
    }
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  latest_baselink_rot_ = quat.normalized().toRotationMatrix();
  latest_baselink_odom_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  have_baselink_odom_feedback_ = true;
}
}  // namespace dracomancer
