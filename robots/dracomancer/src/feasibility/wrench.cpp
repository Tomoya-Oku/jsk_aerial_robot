#include "server.h"

#include "wrench_math.h"
#include <stdexcept>

namespace dracomancer
{
bool ShapeFeasibilityServer::evaluateFixedWrench(
  aerial_robot_model::RobotModel& model, double& force, double& torque)
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
}  // namespace dracomancer
