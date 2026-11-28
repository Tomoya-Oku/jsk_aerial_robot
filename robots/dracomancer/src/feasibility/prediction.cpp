#include "server.h"

#include <dragon/model/full_vectoring_robot_model.h>
#include <stdexcept>

namespace dracomancer
{
bool ShapeFeasibilityServer::evaluateModelFc(double& fc_f_min, double& fc_t_min)
{
  Dragon::FullVectoringRobotModel* dragon_model =
    dynamic_cast<Dragon::FullVectoringRobotModel*>(robot_model_.get());
  if (!dragon_model)
    {
      return evaluateFixedWrench(*robot_model_, fc_f_min, fc_t_min);
    }

  const KDL::JntArray processed_joint =
    dragon_model->getGimbalProcessedJoint<KDL::JntArray>();
  const Eigen::Matrix3d cog_rot =
    dragon_model->getCogDesireOrientation<Eigen::Matrix3d>();
  const auto plan_model = preparePlanModel(*dragon_model, processed_joint, cog_rot);
  return evaluateFixedWrench(*plan_model, fc_f_min, fc_t_min);
}

bool ShapeFeasibilityServer::evaluateOptimizedGimbalFc(double& fc_f_min, double& fc_t_min)
{
  Dragon::FullVectoringRobotModel* dragon_model =
    dynamic_cast<Dragon::FullVectoringRobotModel*>(robot_model_.get());
  if (!dragon_model)
    {
      ROS_WARN_THROTTLE(5.0,
                        "shape_feasibility: optimized_gimbal mode requires Dragon::FullVectoringRobotModel; "
                        "falling back to model fc");
      return evaluateModelFc(fc_f_min, fc_t_min);
    }

  const int rotor_num = dragon_model->getRotorNum();
  const std::vector<int> roll_locked_gimbal = dragon_model->getRollLockedGimbal();
  const KDL::JntArray processed_joint = dragon_model->getGimbalProcessedJoint<KDL::JntArray>();
  const Eigen::Matrix3d cog_rot =
    dragon_model->getCogDesireOrientation<Eigen::Matrix3d>();
  const auto plan_model = preparePlanModel(*dragon_model, processed_joint, cog_rot);
  const std::vector<Eigen::Matrix3d> link_rot = linksRotationFromCog(*plan_model);
  const std::vector<Eigen::Vector3d> rotor_pos =
    plan_model->getRotorsOriginFromCog<Eigen::Vector3d>();

  std::vector<double> locked_roll_angles;
  const auto& joint_index_map = dragon_model->getJointIndexMap();
  for (int i = 0; i < rotor_num; ++i)
    {
      if (roll_locked_gimbal.at(i) == 0)
        {
          continue;
        }
      const std::string joint_name = "gimbal" + std::to_string(i + 1) + "_roll";
      auto it = joint_index_map.find(joint_name);
      if (it == joint_index_map.end())
        {
          throw std::runtime_error("missing joint index for " + joint_name);
        }
      locked_roll_angles.push_back(processed_joint(it->second));
    }

  return evaluateFixedWrench(*plan_model, fc_f_min, fc_t_min);
}

bool ShapeFeasibilityServer::evaluateControllerFeedbackFc(double& fc_f_min, double& fc_t_min)
{
  Dragon::FullVectoringRobotModel* dragon_model =
    dynamic_cast<Dragon::FullVectoringRobotModel*>(robot_model_.get());
  if (!dragon_model)
    {
      ROS_WARN_THROTTLE(5.0,
                        "shape_feasibility: controller mode requires Dragon::FullVectoringRobotModel; "
                        "falling back to model fc");
      return evaluateModelFc(fc_f_min, fc_t_min);
    }

  const int rotor_num = dragon_model->getRotorNum();
  KDL::JntArray feedback_joint = dragon_model->getGimbalProcessedJoint<KDL::JntArray>();
  std::vector<double> locked_roll_angles;
  if (!latestGimbalState(*dragon_model, feedback_joint, locked_roll_angles))
    {
      ROS_WARN_THROTTLE(2.0,
                        "shape_feasibility: controller mode has no fresh roll/pitch gimbal feedback; "
                        "falling back to optimized_gimbal");
      return evaluateOptimizedGimbalFc(fc_f_min, fc_t_min);
    }

  Eigen::Matrix3d cog_rot = dragon_model->getCogDesireOrientation<Eigen::Matrix3d>();
  const auto feedback_rot = latestBaselinkRotation();
  if (feedback_rot.first)
    {
      cog_rot = feedback_rot.second;
    }
  const auto plan_model = preparePlanModel(*dragon_model, feedback_joint, cog_rot);
  const std::vector<Eigen::Matrix3d> link_rot = linksRotationFromCog(*plan_model);
  const std::vector<Eigen::Vector3d> rotor_pos =
    plan_model->getRotorsOriginFromCog<Eigen::Vector3d>();

  // Mirror the controller-side fc check that evaluates all gimbal rolls as
  // locked at the current target roll angles. This is closer to
  // /dragon/debug/fc_*_min than the shape-only nominal-gimbal prediction.
  const std::vector<int> roll_locked_gimbal(rotor_num, 1);
  return evaluateFixedWrench(*plan_model, fc_f_min, fc_t_min);
}

void ShapeFeasibilityServer::setJointPosition(
  const std::map<std::string, unsigned int>& joint_index_map,
  KDL::JntArray& joint_positions, const std::string& joint_name, double value) const
{
  const auto it = joint_index_map.find(joint_name);
  if (it == joint_index_map.end())
    {
      throw std::runtime_error("missing joint index for " + joint_name);
    }
  joint_positions(it->second) = value;
}

KDL::Rotation ShapeFeasibilityServer::matrixToKdlRotation(const Eigen::Matrix3d& rotation)
{
  // KDL::Rotation stores these constructor arguments row by row.
  return KDL::Rotation(rotation(0, 0), rotation(0, 1), rotation(0, 2),
                       rotation(1, 0), rotation(1, 1), rotation(1, 2),
                       rotation(2, 0), rotation(2, 1), rotation(2, 2));
}

boost::shared_ptr<aerial_robot_model::RobotModel>
ShapeFeasibilityServer::preparePlanModel(
  Dragon::FullVectoringRobotModel& dragon_model, const KDL::JntArray& evaluated_joint,
  const Eigen::Matrix3d& cog_rot)
{
  const auto plan_model = dragon_model.getRobotModelForPlan();
  if (!plan_model)
    {
      throw std::runtime_error("full-vectoring planning model is not initialized");
    }
  plan_model->setExtraModuleMap(dragon_model.getExtraModuleMap());
  plan_model->setCogDesireOrientation(matrixToKdlRotation(cog_rot));
  plan_model->updateRobotModel(evaluated_joint);
  return plan_model;
}

std::vector<Eigen::Matrix3d>
ShapeFeasibilityServer::linksRotationFromCog(aerial_robot_model::RobotModel& model)
{
  const auto segment_tf = model.getSegmentsTf();
  const auto baselink_it = segment_tf.find(model.getBaselinkName());
  if (baselink_it == segment_tf.end())
    {
      throw std::runtime_error("missing baselink transform in planning model");
    }
  const KDL::Rotation cog_rot =
    baselink_it->second.M * model.getCogDesireOrientation<KDL::Rotation>().Inverse();

  std::vector<Eigen::Matrix3d> link_rot;
  link_rot.reserve(model.getRotorNum());
  for (int i = 0; i < model.getRotorNum(); ++i)
    {
      const std::string link_name = "link" + std::to_string(i + 1);
      const auto link_it = segment_tf.find(link_name);
      if (link_it == segment_tf.end())
        {
          throw std::runtime_error("missing transform for " + link_name);
        }
      link_rot.push_back(
        aerial_robot_model::kdlToEigen(cog_rot.Inverse() * link_it->second.M));
    }
  return link_rot;
}
}  // namespace dracomancer
