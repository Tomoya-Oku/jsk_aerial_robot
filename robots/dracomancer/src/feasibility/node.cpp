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

#include "server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shape_feasibility_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  try
    {
      dracomancer::ShapeFeasibilityServer server(nh, nhp);
      ros::spin();
    }
  catch (...)
    {
      ROS_ERROR("shape_feasibility_node: shutting down due to initialization error");
      return 1;
    }
  return 0;
}
