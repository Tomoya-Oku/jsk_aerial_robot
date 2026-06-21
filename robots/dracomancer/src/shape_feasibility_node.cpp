/*
 * shape_feasibility_node
 *
 * Predicts the feasible-control force/torque volume inradius (fc_f_min /
 * fc_t_min) of a *candidate* articulated-aerial-robot shape, before the shape
 * command is actually sent. It loads the controlled robot's model (e.g.
 * DRAGON) through pluginlib and evaluates getFeasibleControlF/TMin() for the
 * requested link joint angles. Gimbal joints are filled by the model with its
 * nominal (hovering) angles, so the result is an approximate prediction.
 *
 * Run this node in the controlled robot's namespace (e.g. ns="dragon") so the
 * robot model reads <robot_ns>/robot_description and rotor rosparams.
 */

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/JointState.h>
#include <aerial_robot_model/model/aerial_robot_model.h>
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
        res.fc_f_min = robot_model_->getFeasibleControlFMin();
        res.fc_t_min = robot_model_->getFeasibleControlTMin();
        res.valid = true;
      }
    catch (std::exception& e)
      {
        ROS_WARN_THROTTLE(1.0, "shape_feasibility: model evaluation failed: %s", e.what());
        res.valid = false;
      }
    return true;
  }

  ros::NodeHandle nh_, nhp_;
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  ros::ServiceServer server_;
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
