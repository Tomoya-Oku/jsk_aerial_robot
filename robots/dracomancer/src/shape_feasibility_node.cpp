/*
 * shape_feasibility_node
 *
 * Predicts the feasible-control force/torque volume inradius (fc_f_min /
 * fc_t_min) of a *candidate* articulated-aerial-robot shape, before the shape
 * command is actually sent. It loads the controlled robot's model (e.g.
 * DRAGON) through pluginlib and evaluates getFeasibleControlF/TMin() for the
 * requested link joint angles. Gimbal joints are filled by the model with its
 * nominal (hovering) angles, so the result is an approximate prediction.
 * For DRAGON full-vectoring models, optimized_gimbal mode explicitly evaluates
 * the full-vectoring Fxy/T metrics from the model's gimbal-processed joint
 * state after updateRobotModel().
 *
 * Run this node in the controlled robot's namespace (e.g. ns="dragon") so the
 * robot model reads <robot_ns>/robot_description and rotor rosparams.
 */

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/JointState.h>
#include <aerial_robot_model/model/aerial_robot_model.h>
#include <dragon/model/full_vectoring_robot_model.h>
#include <dracomancer/ShapeFeasibility.h>

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

    prediction_mode_ = nhp_.param<std::string>("prediction_mode", "optimized_gimbal");
    if (prediction_mode_ != "model" && prediction_mode_ != "optimized_gimbal" &&
        prediction_mode_ != "controller")
      {
        ROS_WARN("shape_feasibility: unknown prediction_mode '%s'; using 'optimized_gimbal'",
                 prediction_mode_.c_str());
        prediction_mode_ = "optimized_gimbal";
      }
    ROS_INFO("shape_feasibility: prediction_mode=%s", prediction_mode_.c_str());

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
        res.valid = false;
        return true;
      }

    sensor_msgs::JointState js;
    js.name = req.name;
    js.position = req.position;

    try
      {
        // Unspecified joints (e.g. gimbals) default to 0 and are overwritten by
        // the model's nominal-gimbal processing inside updateRobotModel().
        robot_model_->updateRobotModel(js);
        if (prediction_mode_ == "optimized_gimbal" || prediction_mode_ == "controller")
          {
            evaluateOptimizedGimbalFc(res.fc_f_min, res.fc_t_min);
          }
        else
          {
            res.fc_f_min = robot_model_->getFeasibleControlFMin();
            res.fc_t_min = robot_model_->getFeasibleControlTMin();
          }
        res.valid = true;
      }
    catch (std::exception& e)
      {
        ROS_WARN_THROTTLE(1.0, "shape_feasibility: model evaluation failed: %s", e.what());
        res.valid = false;
      }
    return true;
  }

  void evaluateOptimizedGimbalFc(double& fc_f_min, double& fc_t_min)
  {
    Dragon::FullVectoringRobotModel* dragon_model =
      dynamic_cast<Dragon::FullVectoringRobotModel*>(robot_model_.get());
    if (!dragon_model)
      {
        ROS_WARN_THROTTLE(5.0,
                          "shape_feasibility: optimized_gimbal mode requires Dragon::FullVectoringRobotModel; "
                          "falling back to model fc");
        fc_f_min = robot_model_->getFeasibleControlFMin();
        fc_t_min = robot_model_->getFeasibleControlTMin();
        return;
      }

    if (prediction_mode_ == "controller")
      {
        ROS_WARN_THROTTLE(5.0,
                          "shape_feasibility: controller prediction mode is reserved for the shared controller "
                          "allocator; using optimized_gimbal for now");
      }

    const int rotor_num = dragon_model->getRotorNum();
    const std::vector<int> roll_locked_gimbal = dragon_model->getRollLockedGimbal();
    const std::vector<Eigen::Matrix3d> link_rot =
      dragon_model->getLinksRotationFromCog<Eigen::Matrix3d>();
    const std::vector<Eigen::Vector3d> rotor_pos =
      dragon_model->getRotorsOriginFromCog<Eigen::Vector3d>();
    const Eigen::Matrix3d cog_rot = dragon_model->getCogDesireOrientation<Eigen::Matrix3d>();

    std::vector<double> locked_roll_angles;
    const KDL::JntArray processed_joint = dragon_model->getGimbalProcessedJoint<KDL::JntArray>();
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

    const auto f_min_list =
      dragon_model->calcFeasibleControlFxyDists(roll_locked_gimbal, locked_roll_angles,
                                                rotor_num, link_rot);
    const auto t_min_list =
      dragon_model->calcFeasibleControlTDists(roll_locked_gimbal, locked_roll_angles,
                                              rotor_num, rotor_pos, link_rot, cog_rot);
    fc_f_min = f_min_list.minCoeff();
    fc_t_min = t_min_list.minCoeff();
  }

  ros::NodeHandle nh_, nhp_;
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  ros::ServiceServer server_;
  std::string prediction_mode_;
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
