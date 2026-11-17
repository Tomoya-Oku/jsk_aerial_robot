#pragma once
// Thesis eq:volume-margin and eq:static-allocation, at fixed rotor directions.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dracomancer {
inline void subtractContact(Eigen::VectorXd& required, const Eigen::Matrix3d& axes,
                            const Eigen::Vector3d& lever, const Eigen::VectorXd& wrench)
{
  if (required.size()!=6 || wrench.size()!=6 || !wrench.allFinite() || !lever.allFinite() || !axes.allFinite())
    throw std::runtime_error("invalid contact wrench");
  const Eigen::Vector3d force=axes*wrench.head(3);
  required.head(3)-=force;
  required.tail(3)-=axes*wrench.tail(3)+lever.cross(force);
}

inline double projectedMargin(const Eigen::MatrixXd& generators, const Eigen::VectorXd& target,
                              double lower, double upper)
{
  const int d = generators.rows();
  if ((d != 2 && d != 3) || target.size() != d || !generators.allFinite() ||
      !target.allFinite() || !std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
    throw std::runtime_error("invalid wrench projection");
  const int rank = generators.fullPivLu().rank();
  if (rank < d) {
    // A lower-dimensional projection has no positive ambient-space radius.
    // Keep an on-set target at zero rather than reporting an infinite margin.
    if (rank == 0) return -target.norm();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(generators,Eigen::ComputeFullU);
    Eigen::MatrixXd basis=svd.matrixU().leftCols(rank);
    Eigen::MatrixXd reduced=basis.transpose()*generators;
    Eigen::VectorXd value=basis.transpose()*target;
    double intrinsic;
    if (rank == 1) {
      double lo=0., hi=0.;
      for (int i=0;i<reduced.cols();++i) {
        lo+=std::min(lower*reduced(0,i),upper*reduced(0,i));
        hi+=std::max(lower*reduced(0,i),upper*reduced(0,i));
      }
      intrinsic=std::min(value(0)-lo,hi-value(0));
    } else intrinsic=projectedMargin(reduced,value,lower,upper);
    return std::min(intrinsic,-(target-basis*value).norm());
  }
  double margin = std::numeric_limits<double>::infinity();
  auto facet = [&](Eigen::VectorXd normal) {
    const double norm = normal.norm();
    if (norm < 1e-10) return;
    normal /= norm;
    for (int sign : {-1, 1}) {
      const Eigen::VectorXd n = sign * normal;
      double support = 0.;
      for (int k = 0; k < generators.cols(); ++k) {
        const double v = n.dot(generators.col(k));
        support += std::max(lower*v, upper*v);
      }
      margin = std::min(margin, support-n.dot(target));
    }
  };
  for (int i = 0; i < generators.cols(); ++i) {
    if (d == 2) {
      Eigen::Vector2d n(-generators(1,i), generators(0,i)); facet(n);
    } else {
      for (int j = i+1; j < generators.cols(); ++j) {
        Eigen::Vector3d a = generators.col(i), b = generators.col(j);
        facet(a.cross(b));
      }
    }
  }
  return margin;
}

inline bool commonAllocation(const Eigen::MatrixXd& q, const Eigen::VectorXd& target,
                             double lower, double upper)
{
  // Bounded convex least squares. Only a verified feasible allocation is
  // accepted; an iteration-limit rejection is conservative, not a proof of
  // infeasibility. This also finds solutions when a clipped pseudoinverse fails.
  if (q.rows()!=6 || target.size()!=6 || !q.allFinite() || !target.allFinite()) return false;
  Eigen::VectorXd thrust = q.completeOrthogonalDecomposition().solve(target);
  thrust = thrust.cwiseMax(lower).cwiseMin(upper);
  Eigen::VectorXd error = q*thrust-target;
  for (int iteration=0; iteration<2000; ++iteration) {
    if (error.head(3).norm() <= 1e-5 && error.tail(3).norm() <= 1e-5) return true;
    double change = 0.;
    for (int i=0; i<q.cols(); ++i) {
      const double denominator = q.col(i).squaredNorm();
      if (denominator < 1e-14) continue;
      const double next = std::max(lower, std::min(upper, thrust(i)-q.col(i).dot(error)/denominator));
      const double delta = next-thrust(i);
      error += q.col(i)*delta; thrust(i)=next; change += std::abs(delta);
    }
    if (change < 1e-12) break;
  }
  return error.head(3).norm() <= 1e-5 && error.tail(3).norm() <= 1e-5;
}
} // namespace dracomancer
