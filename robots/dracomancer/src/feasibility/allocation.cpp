#include "server.h"

#include <dragon/model/full_vectoring_robot_model.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace dracomancer
{
bool ShapeFeasibilityServer::evaluateAllocationFc(double& fc_f_min, double& fc_t_min)
{
  Dragon::FullVectoringRobotModel* dragon_model =
    dynamic_cast<Dragon::FullVectoringRobotModel*>(robot_model_.get());
  if (!dragon_model)
    {
      ROS_WARN_THROTTLE(5.0,
                        "shape_feasibility: allocation mode requires Dragon::FullVectoringRobotModel; "
                        "falling back to controller fc");
      return evaluateControllerFeedbackFc(fc_f_min, fc_t_min);
    }

  const int rotor_num = dragon_model->getRotorNum();
  const std::vector<int> roll_locked_gimbal = dragon_model->getRollLockedGimbal();
  std::vector<double> gimbal_nominal_angles = dragon_model->getGimbalNominalAngles();
  std::vector<double> thrust_forces(rotor_num, 0.0);
  std::vector<double> gimbal_angles(2 * rotor_num, 0.0);
  for (int i = 0; i < rotor_num && 2 * i + 1 < static_cast<int>(gimbal_nominal_angles.size()); ++i)
    {
      gimbal_angles.at(2 * i) = gimbal_nominal_angles.at(2 * i);
      gimbal_angles.at(2 * i + 1) = gimbal_nominal_angles.at(2 * i + 1);
    }

  KDL::JntArray gimbal_processed_joint = dragon_model->getGimbalProcessedJoint<KDL::JntArray>();
  Eigen::Matrix3d cog_rot = dragon_model->getCogDesireOrientation<Eigen::Matrix3d>();
  const auto feedback_rot = latestBaselinkRotation();
  if (feedback_rot.first)
    {
      cog_rot = feedback_rot.second;
    }
  const auto plan_model = preparePlanModel(*dragon_model, gimbal_processed_joint, cog_rot);
  const std::vector<Eigen::Matrix3d> link_rot = linksRotationFromCog(*plan_model);
  const int gimbal_lock_num =
    std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);
  Eigen::VectorXd vectoring_forces =
    Eigen::VectorXd::Zero(3 * rotor_num - gimbal_lock_num);

  const Eigen::VectorXd target_acc = allocationTargetAcc(*dragon_model);
  const bool allocation_ok =
    staticHoverAllocation(*dragon_model, *plan_model, target_acc, roll_locked_gimbal,
                          gimbal_nominal_angles, link_rot, gimbal_processed_joint,
                          thrust_forces, gimbal_angles, vectoring_forces);
  if (!allocation_ok)
    {
      ROS_WARN_THROTTLE(2.0,
                        "shape_feasibility: allocation did not converge; using last allocation iterate");
    }

  std::vector<double> locked_roll_angles;
  locked_roll_angles.reserve(rotor_num);
  for (int i = 0; i < rotor_num; ++i)
    {
      locked_roll_angles.push_back(gimbal_angles.at(2 * i));
    }

  const std::vector<int> all_roll_locked(rotor_num, 1);
  const std::vector<Eigen::Vector3d> rotor_pos =
    plan_model->getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Matrix3d> final_link_rot = linksRotationFromCog(*plan_model);

  return evaluateFixedWrench(*plan_model, fc_f_min, fc_t_min);
}

Eigen::VectorXd ShapeFeasibilityServer::allocationTargetAcc(
  const Dragon::FullVectoringRobotModel& dragon_model) const
{
  if (use_allocation_target_acc_param_)
    {
      return allocation_target_acc_;
    }

  Eigen::VectorXd target_acc = Eigen::VectorXd::Zero(6);
  target_acc.head(3) = dragon_model.getGravity3d();
  return target_acc;
}

bool ShapeFeasibilityServer::staticHoverAllocation(
  const Dragon::FullVectoringRobotModel& dragon_model,
  aerial_robot_model::RobotModel& plan_model,
  const Eigen::VectorXd& target_acc,
  const std::vector<int>& roll_locked_gimbal,
  const std::vector<double>& gimbal_nominal_angles,
  const std::vector<Eigen::Matrix3d>& links_rotation_from_cog,
  KDL::JntArray& gimbal_processed_joint,
  std::vector<double>& thrust_forces,
  std::vector<double>& gimbal_angles,
  Eigen::VectorXd& vectoring_forces)
{
  const int rotor_num = plan_model.getRotorNum();
  const int gimbal_lock_num =
    std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);
  const int allocation_dim = 3 * rotor_num - gimbal_lock_num;
  const auto& joint_index_map = plan_model.getJointIndexMap();
  const int max_iteration = std::max(1, allocation_refine_max_iteration_);

  for (int j = 0; j < max_iteration; ++j)
    {
      const std::vector<Eigen::Vector3d> prev_rotors_origin_from_cog =
        plan_model.getRotorsOriginFromCog<Eigen::Vector3d>();
      Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, allocation_dim);
      Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
      wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
      Eigen::Matrix<double, 3, 2> mask;
      mask << 1, 0,
              0, 0,
              0, 1;

      int last_col = 0;
      for (int i = 0; i < rotor_num; ++i)
        {
          wrench_map.block(3, 0, 3, 3) =
            aerial_robot_model::skew(prev_rotors_origin_from_cog.at(i));
          if (roll_locked_gimbal.at(i) == 0)
            {
              full_q_mat.middleCols(last_col, 3) =
                wrench_map * links_rotation_from_cog.at(i);
              last_col += 3;
            }
          else
            {
              full_q_mat.middleCols(last_col, 2) =
                wrench_map * links_rotation_from_cog.at(i) *
                rpyToMatrix(gimbal_nominal_angles.at(2 * i), 0.0, 0.0) * mask;
              last_col += 2;
            }
        }

      Eigen::VectorXd target_wrench = Eigen::VectorXd::Zero(6);
      target_wrench.head(3) = plan_model.getMass() * target_acc.head(3);
      target_wrench.tail(3) =
        plan_model.getInertia<Eigen::Matrix3d>() * target_acc.tail(3);
      vectoring_forces = aerial_robot_model::pseudoinverse(full_q_mat) * target_wrench;

      last_col = 0;
      for (int i = 0; i < rotor_num; ++i)
        {
          if (roll_locked_gimbal.at(i) == 0)
            {
              Eigen::Vector3d f_i = vectoring_forces.segment(last_col, 3);
              thrust_forces.at(i) = f_i.norm();
              applyHoverVectoringFallback(dragon_model, last_col + 2, f_i.z());
              const double gimbal_roll = atan2(-f_i.y(), f_i.z());
              const double gimbal_pitch =
                atan2(f_i.x(), -f_i.y() * sin(gimbal_roll) + f_i.z() * cos(gimbal_roll));
              gimbal_angles.at(2 * i) = gimbal_roll;
              gimbal_angles.at(2 * i + 1) = gimbal_pitch;
              last_col += 3;
            }
          else
            {
              Eigen::Vector2d f_i = vectoring_forces.segment(last_col, 2);
              thrust_forces.at(i) = f_i.norm();
              applyHoverVectoringFallback(dragon_model, last_col + 1, f_i(1));
              gimbal_angles.at(2 * i) = gimbal_nominal_angles.at(2 * i);
              gimbal_angles.at(2 * i + 1) = atan2(f_i(0), f_i(1));
              last_col += 2;
            }
        }

      for (int i = 0; i < rotor_num; ++i)
        {
          const std::string suffix = std::to_string(i + 1);
          setJointPosition(joint_index_map, gimbal_processed_joint,
                           "gimbal" + suffix + "_roll", gimbal_angles.at(2 * i));
          setJointPosition(joint_index_map, gimbal_processed_joint,
                           "gimbal" + suffix + "_pitch", gimbal_angles.at(2 * i + 1));
        }
      // The plain planning model honors these exact gimbal angles. Updating
      // FullVectoringRobotModel here would replace them with its own nominal
      // allocation, making the returned fc and stability gate use different Q.
      plan_model.updateRobotModel(gimbal_processed_joint);

      const std::vector<Eigen::Vector3d> rotors_origin_from_cog =
        plan_model.getRotorsOriginFromCog<Eigen::Vector3d>();
      double max_diff = 0.0;
      for (int i = 0; i < rotor_num; ++i)
        {
          max_diff = std::max(max_diff,
                              (rotors_origin_from_cog.at(i) -
                               prev_rotors_origin_from_cog.at(i)).norm());
        }
      if (max_diff < allocation_refine_threshold_)
        {
          return true;
        }
    }
  return false;
}

void ShapeFeasibilityServer::applyHoverVectoringFallback(
  const Dragon::FullVectoringRobotModel& dragon_model, int hover_index, double& force_component) const
{
  if (allocation_start_rp_integration_)
    {
      return;
    }
  const auto& hover_vectoring_f = dragon_model.getHoverVectoringF();
  if (hover_index >= 0 && hover_index < hover_vectoring_f.size())
    {
      force_component = hover_vectoring_f(hover_index);
    }
}
}  // namespace dracomancer
