// Gazebo ODE reports each body's wrench in that body's link frame, about
// its CoM. Select the robot body explicitly; never negate a wall-side torque.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <cmath>
#include <memory>

namespace gazebo
{
class WallContact : public SensorPlugin
{
  sensors::ContactSensorPtr sensor_;
  physics::WorldPtr world_;
  event::ConnectionPtr connection_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_;
  std::string robot_, link_, frame_;

public:
  void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override
  {
    sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(parent);
    if (!sensor_ || !ros::isInitialized())
      gzthrow("WallContact requires a contact sensor and gazebo_ros");
    world_ = physics::get_world(parent->WorldName());
    robot_ = sdf->Get<std::string>("robot");
    link_ = sdf->Get<std::string>("link");
    frame_ = robot_ + "/" + link_;
    nh_.reset(new ros::NodeHandle("/" + robot_ + "/contact"));
    pub_ = nh_->advertise<geometry_msgs::WrenchStamped>("wrench", 1);
    connection_ = sensor_->ConnectUpdated(std::bind(&WallContact::update, this));
    sensor_->SetActive(true);
  }

  void update()
  {
    const auto model = world_->ModelByName(robot_);
    if (!model) return;
    physics::CollisionPtr head;
    // URDF fixed-joint reduction merges link1 into root in the real DRAGON.
    // Locate its retained collision rather than assuming link1 is a physics body.
    for (const auto& body : model->GetLinks()) {
      for (const auto& collision : body->GetCollisions()) {
        const auto name = collision->GetName();
        const auto suffix = "__" + link_ + "_collision";
        if ((body->GetName() == link_) || name == link_ + "_collision" ||
            (name.size() >= suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)) {
          if (boost::dynamic_pointer_cast<physics::BoxShape>(collision->GetShape())) head = collision;
        }
      }
    }
    if (!head) return;
    const auto link = head->GetLink();
    const auto box = boost::dynamic_pointer_cast<physics::BoxShape>(head->GetShape());
    // Existing DRAGON link1 collision: its origin is at the box centre; the
    // original link1 origin is the head face at negative local X.
    const auto head_in_body = head->RelativePose() * ignition::math::Pose3d(-box->Size().X() / 2, 0, 0, 0, 0, 0);
    const auto contacts = sensor_->Contacts();
    ignition::math::Vector3d force = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d torque = ignition::math::Vector3d::Zero;
    const auto pose = link->WorldPose() * head_in_body;
    const auto com = link->GetInertial()->Pose().Pos();
    for (int i = 0; i < contacts.contact_size(); ++i)
    {
      const auto& contact = contacts.contact(i);
      const bool first = contact.collision1() == head->GetScopedName();
      const bool second = contact.collision2() == head->GetScopedName();
      if (!first && !second) continue;
      for (int j = 0; j < contact.wrench_size() && j < contact.position_size(); ++j)
      {
        const auto& point = contact.position(j);
        const auto local = pose.Rot().RotateVectorReverse(
            ignition::math::Vector3d(point.x(), point.y(), point.z()) - pose.Pos());
        ROS_DEBUG_STREAM_THROTTLE(1, "Head contact in link1: " << local);
        // Existing link1 head face is x=0. A 1 mm numerical tolerance permits
        // ODE surface penetration; side, tail and rotor collisions are excluded.
        if (std::abs(local.X()) > 0.001) continue;
        const auto& wrench = first ? contact.wrench(j).body_1_wrench()
                                   : contact.wrench(j).body_2_wrench();
        ignition::math::Vector3d f(wrench.force().x(), wrench.force().y(), wrench.force().z());
        ignition::math::Vector3d t(wrench.torque().x(), wrench.torque().y(), wrench.torque().z());
        force += head_in_body.Rot().RotateVectorReverse(f);
        torque += head_in_body.Rot().RotateVectorReverse(t + (com - head_in_body.Pos()).Cross(f));
      }
    }
    // Publish explicit zero on loss of contact. Paused physics stops these
    // updates, which the ROS wall-clock watchdog detects independently.
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp.fromSec(world_->SimTime().Double());
    msg.header.frame_id = frame_;
    msg.wrench.force.x = force.X(); msg.wrench.force.y = force.Y(); msg.wrench.force.z = force.Z();
    msg.wrench.torque.x = torque.X(); msg.wrench.torque.y = torque.Y(); msg.wrench.torque.z = torque.Z();
    pub_.publish(msg);
  }
};
GZ_REGISTER_SENSOR_PLUGIN(WallContact)
}
