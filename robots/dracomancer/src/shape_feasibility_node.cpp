/*
 * shape_feasibility_node
 *
 * Predicts the feasible-control force/torque volume inradius (fc_f_min /
 * fc_t_min) and coupled static-wrench feasibility of a *candidate*
 * articulated-aerial-robot shape, before the shape command is actually sent.
 * It loads the controlled robot's model (e.g.
 * DRAGON) through pluginlib. Prediction modes choose a gimbal/attitude context;
 * all margins and the common bounded allocation then use the same fixed Q.
 * This differs from the former variable-pitch Fxy metric. Context fields let
 * the caller freeze gimbals and attitude for the rate-limited recheck.
 *
 * Run this node in the controlled robot's namespace (e.g. ns="dragon") so the
 * robot model reads <robot_ns>/robot_description and rotor rosparams.
 */

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_model/utils/math_utils.h>
#include <dragon/model/full_vectoring_robot_model.h>
#include <dracomancer/ShapeFeasibility.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "wrench_math.h"

class ShapeFeasibilityServer
{
public:
  ShapeFeasibilityServer(ros::NodeHandle nh, ros::NodeHandle nhp)
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

private:
  bool checkShape(dracomancer::ShapeFeasibility::Request& req,
                  dracomancer::ShapeFeasibility::Response& res)
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

  bool evaluateModelFc(double& fc_f_min, double& fc_t_min)
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

  bool evaluateOptimizedGimbalFc(double& fc_f_min, double& fc_t_min)
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

  bool evaluateControllerFeedbackFc(double& fc_f_min, double& fc_t_min)
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

  bool evaluateAllocationFc(double& fc_f_min, double& fc_t_min)
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

  bool evaluateFixedWrench(aerial_robot_model::RobotModel& model, double& force, double& torque)
  {
    const Eigen::MatrixXd q=model.calcWrenchMatrixOnCoG();
    const auto frames=model.getSegmentsTf();
    auto fc=frames.find("fc");
    if (fc==frames.end()) throw std::runtime_error("missing FC frame");
    const auto cog=model.getCog<KDL::Frame>();
    const Eigen::Matrix3d c_fc=aerial_robot_model::kdlToEigen(cog.M.Inverse()*fc->second.M);
    auto tip=frames.find("link1");
    if (tip==frames.end()) throw std::runtime_error("missing contact link");
    const Eigen::Matrix3d c_e=aerial_robot_model::kdlToEigen(cog.M.Inverse()*tip->second.M);
    const KDL::Vector point=cog.M.Inverse()*(tip->second.p-cog.p);
    const Eigen::Vector3d r(point.x(),point.y(),point.z());
    const Eigen::Matrix3d wrench_rotation=external_in_link1_ ? c_e : c_fc;
    Eigen::VectorXd required=Eigen::VectorXd::Zero(6);
    dracomancer::subtractContact(required,wrench_rotation,r,external_wrench_);
    for (size_t i=0;i<contact_links_.size();++i) {
      auto contact=frames.find(contact_links_[i]);
      if (contact==frames.end()) throw std::runtime_error("unknown contact link");
      Eigen::Vector3d offset(contact_points_[3*i],contact_points_[3*i+1],contact_points_[3*i+2]);
      KDL::Vector local(offset.x(),offset.y(),offset.z());
      KDL::Vector point=cog.M.Inverse()*(contact->second*local-cog.p);
      Eigen::Vector3d lever(point.x(),point.y(),point.z());
      Eigen::Matrix3d axes=contact_local_[i] ? aerial_robot_model::kdlToEigen(cog.M.Inverse()*contact->second.M) : c_fc;
      Eigen::VectorXd wrench=Eigen::Map<const Eigen::VectorXd>(&contact_wrenches_[6*i],6);
      dracomancer::subtractContact(required,axes,lever,wrench);
    }
    // This model stores the upward gravity compensation vector, not physical g.
    required.head(3) += model.getMass()*model.getGravity3d();
    const double lower=model.getThrustLowerLimit(), upper=model.getThrustUpperLimit();
    force=dracomancer::projectedMargin(q.topRows(force_dimensions_),required.head(force_dimensions_),lower,upper);
    torque=dracomancer::projectedMargin(q.bottomRows(3),required.tail(3),lower,upper);
    evaluated_gimbals_.clear(); evaluated_rotation_.clear();
    const auto& index=model.getJointIndexMap();
    const auto joints=model.getJointPositions();
    for (int i=0;i<model.getRotorNum();++i)
      for (const std::string axis : {"roll","pitch"}) {
        auto it=index.find("gimbal"+std::to_string(i+1)+"_"+axis);
        if (it!=index.end()) evaluated_gimbals_.push_back(joints(it->second));
      }
    Eigen::Matrix3d orientation=model.getCogDesireOrientation<Eigen::Matrix3d>();
    for (int i=0;i<9;++i) evaluated_rotation_.push_back(orientation(i/3,i%3));
    fc_to_cog_.clear();
    for (int i=0;i<9;++i) fc_to_cog_.push_back(c_fc(i/3,i%3));
    return dracomancer::commonAllocation(q,required,lower,upper);
  }

  Eigen::VectorXd allocationTargetAcc(const Dragon::FullVectoringRobotModel& dragon_model) const
  {
    if (use_allocation_target_acc_param_)
      {
        return allocation_target_acc_;
      }

    Eigen::VectorXd target_acc = Eigen::VectorXd::Zero(6);
    target_acc.head(3) = dragon_model.getGravity3d();
    return target_acc;
  }

  bool staticHoverAllocation(const Dragon::FullVectoringRobotModel& dragon_model,
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

  void applyHoverVectoringFallback(const Dragon::FullVectoringRobotModel& dragon_model,
                                   int hover_index,
                                   double& force_component) const
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

  void setJointPosition(const std::map<std::string, unsigned int>& joint_index_map,
                        KDL::JntArray& joint_positions,
                        const std::string& joint_name,
                        double value) const
  {
    const auto it = joint_index_map.find(joint_name);
    if (it == joint_index_map.end())
      {
        throw std::runtime_error("missing joint index for " + joint_name);
      }
    joint_positions(it->second) = value;
  }

  static KDL::Rotation matrixToKdlRotation(const Eigen::Matrix3d& rotation)
  {
    // KDL::Rotation stores these constructor arguments row by row.
    return KDL::Rotation(rotation(0, 0), rotation(0, 1), rotation(0, 2),
                         rotation(1, 0), rotation(1, 1), rotation(1, 2),
                         rotation(2, 0), rotation(2, 1), rotation(2, 2));
  }

  boost::shared_ptr<aerial_robot_model::RobotModel>
  preparePlanModel(Dragon::FullVectoringRobotModel& dragon_model,
                   const KDL::JntArray& evaluated_joint,
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
  linksRotationFromCog(aerial_robot_model::RobotModel& model)
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

  bool latestGimbalState(const Dragon::FullVectoringRobotModel& dragon_model,
                         KDL::JntArray& feedback_joint,
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

  std::pair<bool, Eigen::Matrix3d> latestBaselinkRotation()
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

  static Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw)
  {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }

  void gimbalCb(const sensor_msgs::JointStateConstPtr& msg)
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

  void baselinkRpyCb(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    latest_baselink_rpy_[0] = msg->vector.x;
    latest_baselink_rpy_[1] = msg->vector.y;
    latest_baselink_rpy_[2] = msg->vector.z;
    latest_baselink_rpy_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_baselink_rpy_feedback_ = true;
  }

  void baselinkOdomCb(const nav_msgs::OdometryConstPtr& msg)
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shape_feasibility_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  try
    {
      ShapeFeasibilityServer server(nh, nhp);
      ros::spin();
    }
  catch (...)
    {
      ROS_ERROR("shape_feasibility_node: shutting down due to initialization error");
      return 1;
    }
  return 0;
}
