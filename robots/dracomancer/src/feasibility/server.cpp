#include "server.h"

#include <dragon/model/full_vectoring_robot_model.h>
#include <cmath>
#include <stdexcept>

namespace dracomancer
{
ShapeFeasibilityServer::ShapeFeasibilityServer(ros::NodeHandle nh, ros::NodeHandle nhp)
  : nh_(nh), nhp_(nhp),
    robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
{
  // Plugin name: prefer the controlled robot's own param, allow private override.
  std::string plugin_name;
  if (!nhp_.getParam("robot_model_plugin_name", plugin_name) &&
      !nh_.getParam("robot_model_plugin_name", plugin_name))
    {
      plugin_name = "dragon/full_vectoring_robot_model";
      ROS_WARN("shape_feasibility: robot_model_plugin_name not set, using default '%s'",
               plugin_name.c_str());
    }

  try
    {
      robot_model_ = robot_model_loader_.createInstance(plugin_name);
    }
  catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("shape_feasibility: failed to load robot model plugin '%s': %s",
                plugin_name.c_str(), ex.what());
      throw;
    }

  ROS_INFO("shape_feasibility: loaded robot model plugin '%s' (ns=%s)",
           plugin_name.c_str(), nh_.getNamespace().c_str());

  prediction_mode_ = nhp_.param<std::string>("prediction_mode", "allocation");
  force_dimensions_ = nhp_.param<int>("force_dimensions", 2);
  if (force_dimensions_ != 2 && force_dimensions_ != 3)
    throw std::runtime_error("force_dimensions must be 2 or 3");
  if (prediction_mode_ != "model" && prediction_mode_ != "optimized_gimbal" &&
      prediction_mode_ != "controller" && prediction_mode_ != "allocation")
    {
      ROS_WARN("shape_feasibility: unknown prediction_mode '%s'; using 'optimized_gimbal'",
               prediction_mode_.c_str());
      prediction_mode_ = "optimized_gimbal";
    }

  gimbal_feedback_topic_ =
    nhp_.param<std::string>("gimbal_feedback_topic", "gimbals_ctrl");
  baselink_odom_topic_ =
    nhp_.param<std::string>("baselink_odom_topic", "uav/baselink/odom");
  baselink_rpy_topic_ =
    nhp_.param<std::string>("baselink_rpy_topic", "final_target_baselink_rpy");
  use_gimbal_feedback_ = nhp_.param<bool>("use_gimbal_feedback", true);
  use_baselink_odom_feedback_ = nhp_.param<bool>("use_baselink_odom_feedback", false);
  use_baselink_rpy_feedback_ = nhp_.param<bool>("use_baselink_rpy_feedback", true);
  feedback_timeout_ = nhp_.param<double>("feedback_timeout", 0.25);
  allocation_refine_max_iteration_ =
    nhp_.param<int>("allocation_refine_max_iteration",
                    nh_.param<int>("controller/allocation_refine_max_iteration", 5));
  allocation_refine_threshold_ =
    nhp_.param<double>("allocation_refine_threshold",
                       nh_.param<double>("controller/allocation_refine_threshold", 0.0001));
  allocation_start_rp_integration_ =
    nhp_.param<bool>("allocation_start_rp_integration", true);
  const auto target_acc_param =
    nhp_.param<std::vector<double> >("allocation_target_acc", std::vector<double>());
  if (!target_acc_param.empty())
    {
      if (target_acc_param.size() != 6)
        {
          ROS_WARN("shape_feasibility: allocation_target_acc must have 6 elements; "
                   "falling back to gravity hover target");
        }
      else
        {
          allocation_target_acc_ = Eigen::Map<const Eigen::VectorXd>(target_acc_param.data(), 6);
          use_allocation_target_acc_param_ = true;
        }
    }
  if (use_gimbal_feedback_)
    {
      gimbal_sub_ = nh_.subscribe(gimbal_feedback_topic_, 1,
                                  &ShapeFeasibilityServer::gimbalCb, this);
    }
  if (use_baselink_odom_feedback_)
    {
      baselink_odom_sub_ = nh_.subscribe(baselink_odom_topic_, 1,
                                         &ShapeFeasibilityServer::baselinkOdomCb, this);
    }
  if (use_baselink_rpy_feedback_)
    {
      baselink_rpy_sub_ = nh_.subscribe(baselink_rpy_topic_, 1,
                                        &ShapeFeasibilityServer::baselinkRpyCb, this);
    }
  ROS_INFO("shape_feasibility: prediction_mode=%s, gimbal_feedback=%s (%s), "
           "baselink_rpy_feedback=%s (%s), baselink_odom_feedback=%s (%s), "
           "allocation_iter=%d, allocation_threshold=%.6f",
           prediction_mode_.c_str(),
           use_gimbal_feedback_ ? "true" : "false", gimbal_feedback_topic_.c_str(),
           use_baselink_rpy_feedback_ ? "true" : "false", baselink_rpy_topic_.c_str(),
           use_baselink_odom_feedback_ ? "true" : "false", baselink_odom_topic_.c_str(),
           allocation_refine_max_iteration_, allocation_refine_threshold_);

  server_ = nhp_.advertiseService("check_shape", &ShapeFeasibilityServer::checkShape, this);
}

bool ShapeFeasibilityServer::checkShape(ShapeFeasibility::Request& req,
                                       ShapeFeasibility::Response& res)
{
  if (req.name.size() != req.position.size())
    {
      ROS_WARN_THROTTLE(1.0, "shape_feasibility: name/position size mismatch (%zu vs %zu)",
                        req.name.size(), req.position.size());
      res.stability_ok = false;
      res.valid = false;
      return true;
    }

  sensor_msgs::JointState js;
  js.name = req.name;
  js.position = req.position;

  try
    {
      std::map<std::string, bool> names;
      for (size_t i=0; i<req.name.size(); ++i) {
        if (!std::isfinite(req.position[i]) || !names.emplace(req.name[i], true).second)
          throw std::runtime_error("nonfinite or duplicate candidate joint");
        const auto joint=robot_model_->getUrdfModel().getJoint(req.name[i]);
        if (!joint) throw std::runtime_error("unknown candidate joint");
        if (joint->type==urdf::Joint::REVOLUTE && joint->limits &&
            (req.position[i]<joint->limits->lower-1e-8 || req.position[i]>joint->limits->upper+1e-8))
          throw std::runtime_error("candidate joint outside URDF limits");
      }
      if (dynamic_cast<Dragon::FullVectoringRobotModel*>(robot_model_.get()))
        for (int i=1;i<=3;++i)
          for (const std::string axis:{"pitch","yaw"})
            if (!names.count("joint"+std::to_string(i)+"_"+axis))
              throw std::runtime_error("incomplete DRAGON candidate");
      external_wrench_ = Eigen::VectorXd::Zero(6);
      external_in_link1_ = req.wrench_in_link1;
      contact_links_=req.contact_link;
      contact_points_=req.contact_point;
      contact_wrenches_=req.contact_wrench;
      contact_local_.clear();
      for (bool local:req.contact_local) contact_local_.push_back(local);
      if (contact_points_.size()!=3*contact_links_.size() || contact_wrenches_.size()!=6*contact_links_.size()
          || contact_local_.size()!=contact_links_.size()) throw std::runtime_error("invalid contact arrays");
      for (double value:contact_points_) if (!std::isfinite(value)) throw std::runtime_error("invalid contact point");
      for (double value:contact_wrenches_) if (!std::isfinite(value)) throw std::runtime_error("invalid contact wrench");
      if (!req.external_wrench.empty()) {
        if (req.external_wrench.size()!=6) throw std::runtime_error("external wrench requires 6 components");
        external_wrench_ = Eigen::Map<const Eigen::VectorXd>(req.external_wrench.data(),6);
        if (!external_wrench_.allFinite()) throw std::runtime_error("nonfinite external wrench");
      }
      // Unspecified joints (e.g. gimbals) default to 0 and are overwritten by
      // the model's nominal-gimbal processing inside updateRobotModel().
      robot_model_->updateRobotModel(js);
      if (!req.evaluation_gimbals.empty() || !req.evaluation_rotation.empty()) {
        auto* dragon = dynamic_cast<Dragon::FullVectoringRobotModel*>(robot_model_.get());
        if (!dragon || req.evaluation_gimbals.size()!=size_t(2*dragon->getRotorNum()) || req.evaluation_rotation.size()!=9)
          throw std::runtime_error("incomplete evaluation context");
        KDL::JntArray joints=dragon->getGimbalProcessedJoint<KDL::JntArray>();
        for (int i=0;i<dragon->getRotorNum();++i) {
          setJointPosition(dragon->getJointIndexMap(),joints,"gimbal"+std::to_string(i+1)+"_roll",req.evaluation_gimbals[2*i]);
          setJointPosition(dragon->getJointIndexMap(),joints,"gimbal"+std::to_string(i+1)+"_pitch",req.evaluation_gimbals[2*i+1]);
        }
        Eigen::Matrix3d orientation;
        for (int i=0;i<9;++i) orientation(i/3,i%3)=req.evaluation_rotation[i];
        if (!orientation.allFinite() || !(orientation.transpose()*orientation).isApprox(Eigen::Matrix3d::Identity(),1e-7)
            || std::abs(orientation.determinant()-1.)>1e-7) throw std::runtime_error("invalid evaluation rotation");
        auto plan=preparePlanModel(*dragon,joints,orientation);
        res.stability_ok=evaluateFixedWrench(*plan,res.fc_f_min,res.fc_t_min);
      }
      else if (prediction_mode_ == "allocation")
        {
          res.stability_ok = evaluateAllocationFc(res.fc_f_min, res.fc_t_min);
        }
      else if (prediction_mode_ == "controller")
        {
          res.stability_ok = evaluateControllerFeedbackFc(res.fc_f_min, res.fc_t_min);
        }
      else if (prediction_mode_ == "optimized_gimbal")
        {
          res.stability_ok = evaluateOptimizedGimbalFc(res.fc_f_min, res.fc_t_min);
        }
      else
        {
          res.stability_ok = evaluateModelFc(res.fc_f_min, res.fc_t_min);
        }
      res.valid = true;
      res.evaluation_gimbals = evaluated_gimbals_;
      res.evaluation_rotation = evaluated_rotation_;
      res.force_dimensions = force_dimensions_;
      res.fc_to_cog = fc_to_cog_;
    }
  catch (std::exception& e)
    {
      ROS_WARN_THROTTLE(1.0, "shape_feasibility: model evaluation failed: %s", e.what());
      res.stability_ok = false;
      res.valid = false;
    }
  return true;
}
}  // namespace dracomancer
