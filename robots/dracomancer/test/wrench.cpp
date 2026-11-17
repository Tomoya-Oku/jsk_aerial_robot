#include "../src/wrench_math.h"
#include <cassert>
#include <iostream>
int main() {
  Eigen::VectorXd load=Eigen::VectorXd::Zero(6),contact=Eigen::VectorXd::Zero(6);
  contact(1)=3.;contact(5)=2.;
  dracomancer::subtractContact(load,Eigen::Matrix3d::Identity(),Eigen::Vector3d(1,0,0),contact);
  assert(std::abs(load(1)+3.)<1e-10 && std::abs(load(5)+5.)<1e-10);
  load.setZero();contact.setZero();contact(0)=1.;
  dracomancer::subtractContact(load,Eigen::Matrix3d::Identity(),Eigen::Vector3d(-.5,0,0),contact);
  contact(0)=-1.;
  dracomancer::subtractContact(load,Eigen::Matrix3d::Identity(),Eigen::Vector3d(.5,0,0),contact);
  assert(load.norm()<1e-10);
  Eigen::Matrix2d box=Eigen::Matrix2d::Identity();
  Eigen::Vector2d p(.5,.5);
  assert(std::abs(dracomancer::projectedMargin(box,p,0,1)-.5)<1e-10);
  p<<1.2,.5;
  assert(std::abs(dracomancer::projectedMargin(box,p,0,1)+.2)<1e-10);
  Eigen::MatrixXd line=Eigen::MatrixXd::Zero(2,1);line(0,0)=1.;
  p<<.5,0.;assert(std::abs(dracomancer::projectedMargin(line,p,0,1))<1e-10);
  p<<.5,.2;assert(dracomancer::projectedMargin(line,p,0,1)<0.);
  Eigen::MatrixXd q(6,3); q.topRows(3)=Eigen::Matrix3d::Identity(); q.bottomRows(3)=Eigen::Matrix3d::Identity();
  Eigen::VectorXd target=Eigen::VectorXd::Constant(6,.2);
  assert(dracomancer::commonAllocation(q,target,0,1));
  target.tail(3).setConstant(.8);
  assert(dracomancer::projectedMargin(q.topRows(3),target.head(3),0,1)>0);
  assert(dracomancer::projectedMargin(q.bottomRows(3),target.tail(3),0,1)>0);
  assert(!dracomancer::commonAllocation(q,target,0,1));
  q=Eigen::MatrixXd::Zero(6,2); q(0,0)=1.; q(0,1)=2.;
  target=Eigen::VectorXd::Zero(6); target(0)=2.8;
  assert(dracomancer::commonAllocation(q,target,0,1));
  std::cout << "PASS: signed XY/XYZ margins and common bounded allocation\n";
}
