#include <aerial_robot_model/model/aerial_robot_model.h>

#include <gtest/gtest.h>

#include <cmath>

namespace {

class TestableRobotModel : public aerial_robot_model::RobotModel
{
public:
  using RobotModel::calcFeasibleControlDists;
  using RobotModel::calcFeasibleControlMin;
  using RobotModel::isValidFeasibleControlPlane;
};

TEST(FeasibleControlMargin, IgnoresParallelForceGeneratorPairs)
{
  const std::vector<Eigen::Vector3d> generators = {
    Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitZ(),
  };
  const Eigen::Vector3d center(1.0, 0.5, 0.5);

  const Eigen::VectorXd distances =
    TestableRobotModel::calcFeasibleControlDists(generators, 1.0, center);

  EXPECT_TRUE(std::isinf(distances(0)));
  EXPECT_NEAR(TestableRobotModel::calcFeasibleControlMin(distances), 0.5, 1e-12);
}

TEST(FeasibleControlMargin, IgnoresParallelTorqueGeneratorPairs)
{
  const std::vector<Eigen::Vector3d> generators = {
    Eigen::Vector3d::UnitX(),
    -Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitY(),
    -Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitZ(),
    -Eigen::Vector3d::UnitZ(),
  };

  const Eigen::VectorXd distances =
    TestableRobotModel::calcFeasibleControlDists(generators, 1.0, Eigen::Vector3d::Zero());

  EXPECT_NEAR(TestableRobotModel::calcFeasibleControlMin(distances), 1.0, 1e-12);
}

TEST(FeasibleControlMargin, KeepsNegativeForceMarginOutsideVolume)
{
  std::vector<Eigen::Vector3d> generators = {
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(-1.0, -1.0, -1.0),
    Eigen::Vector3d(-1.0, 1.0, -1.0),
    Eigen::Vector3d(1.0, -1.0, -1.0),
    Eigen::Vector3d(1.0, 2.0, -1.0),
  };
  for (auto& generator : generators) generator.normalize();
  const Eigen::Vector3d gravity_force(0.0, 0.0, 0.25);

  const Eigen::VectorXd distances =
    TestableRobotModel::calcFeasibleControlDists(generators, 1.0, gravity_force);

  EXPECT_NEAR(TestableRobotModel::calcFeasibleControlMin(distances), -0.25, 1e-12);
}

TEST(FeasibleControlMargin, UsesScaleIndependentParallelCheck)
{
  const Eigen::Vector3d small_x = 1e-6 * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d small_y = 1e-6 * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d near_parallel(1.0, 1e-8, 0.0);

  EXPECT_TRUE(TestableRobotModel::isValidFeasibleControlPlane(small_x, small_y));
  EXPECT_TRUE(TestableRobotModel::isValidFeasibleControlPlane(Eigen::Vector3d::UnitX(), near_parallel));
  EXPECT_FALSE(TestableRobotModel::isValidFeasibleControlPlane(small_x, 2.0 * small_x));
  EXPECT_FALSE(TestableRobotModel::isValidFeasibleControlPlane(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()));
}

TEST(FeasibleControlMargin, ReturnsZeroForPlanarVolume)
{
  const std::vector<Eigen::Vector3d> generators = {
    Eigen::Vector3d::UnitX(),
    -Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitY(),
    -Eigen::Vector3d::UnitY(),
  };

  const Eigen::VectorXd distances =
    TestableRobotModel::calcFeasibleControlDists(generators, 1.0, Eigen::Vector3d::Zero());

  EXPECT_GT(distances.array().isFinite().count(), 0);
  EXPECT_DOUBLE_EQ(TestableRobotModel::calcFeasibleControlMin(distances), 0.0);
}

TEST(FeasibleControlMargin, ReturnsZeroWithoutValidPlane)
{
  const std::vector<Eigen::Vector3d> generators = {
    Eigen::Vector3d::UnitX(),
    -Eigen::Vector3d::UnitX(),
    2.0 * Eigen::Vector3d::UnitX(),
  };

  const Eigen::VectorXd distances =
    TestableRobotModel::calcFeasibleControlDists(generators, 1.0, Eigen::Vector3d::Zero());

  EXPECT_DOUBLE_EQ(TestableRobotModel::calcFeasibleControlMin(distances), 0.0);
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
