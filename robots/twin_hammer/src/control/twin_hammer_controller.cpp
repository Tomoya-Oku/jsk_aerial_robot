#include <twin_hammer/control/twin_hammer_controller.h>

using namespace aerial_robot_control;

namespace
{
  double GimbalRoundPolynominal(double x, double r)
  {
    return (2/(r*r)) * pow(x,3) - (1/pow(r,4)) * pow(x,5);
  }

  Eigen::VectorXd GaussianEq(const Eigen::VectorXd& vars, double r, double e)
  {
    double a = vars(0);
    double b = vars(1);

    Eigen::VectorXd eq(2);
    // y = r when x = r
    eq(0) = a*r*(1-exp(-e*r*r)) + b*pow(r,3) - r;
    // dy/dx = 1 when x = r
    eq(1) = a*(1-exp(-e*r*r)) + 2*e*r*r*exp(-e*r*r) + 3*b*r*r - 1;
    return eq;
  }

  Eigen::VectorXd SolveGaussian(double r, double e)
  {
    Eigen::VectorXd vars(2);
    vars << 1.0, 1.0; // initial value
    const double tol = 1e-6;
    const int max_itr = 100;
    int itr = 0;

    while(itr < max_itr){
      Eigen::VectorXd f = GaussianEq(vars, r, e);
      Eigen::MatrixXd J(2,2); //Jacobian
      double h = 1e-8;

      for(int i=0; i<2;i++){
        Eigen::VectorXd vars_plus_h = vars;
        vars_plus_h(i) += h;
        Eigen::VectorXd f_plus_h = GaussianEq(vars_plus_h, r, e);
        J.col(i) = (f_plus_h-f)/h;
      }

      Eigen::VectorXd delta = J.fullPivLu().solve(-f);
      vars += delta;

      if(delta.norm() < tol){
        break;
      }
      ++itr;
    }

    if(itr == max_itr){
      std::cerr << "Gaussian solved value haven't converged" << std::endl;
    }
    return vars;
  }

  double GimbalRoundGaussian(double x, double r, double e)
  {
    Eigen::VectorXd vars = SolveGaussian(r, e);
    double a = vars(0);
    double b = vars(1);
    return a*x*(1-exp(-e*x*x)) + b*pow(x,3);
  }
}

TwinHammerController::TwinHammerController():
  PoseLinearController()
{
}

void TwinHammerController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                  double ctrl_loop_rate
                                  )
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  twin_hammer_model_ = boost::dynamic_pointer_cast<TwinHammerModel>(robot_model);

  target_base_thrust_.resize(motor_num_, 0.0);
  target_gimbal_angles_.resize(motor_num_, 0.01);
  prev_gimbal_angles_.resize(motor_num_, 0.0);
  gimbal_states_angles_.resize(motor_num_, 0.0);
  target_wrench_acc_cog_ = Eigen::VectorXd::Zero(6);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbal_states_sub_ = nh_.subscribe("joint_states", 1, &TwinHammerController::GimbalStatesCallback, this);
  haptics_switch_sub_ = nh_.subscribe("haptics_switch", 1, &TwinHammerController::HapticsSwitchCallback, this);
  haptics_wrench_sub_ = nh_.subscribe("haptics_wrench", 1, &TwinHammerController::HapticsWrenchCallback, this);

  target_wrench_acc_cog_ = Eigen::VectorXd::Zero(6);
  haptics_switch_ = false;
  haptics_force_ = Eigen::Vector3d::Zero();
  haptics_torque_ = Eigen::Vector3d::Zero();
  filtered_gimbal_1_roll_ = 0.0;
  filtered_gimbal_2_roll_ = 0.0;

  rosParamInit();
}

void TwinHammerController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "use_haptics", use_haptics_flag_, true);
  getParam<bool>(control_nh, "use_polynominal", use_polynominal_, true);
  getParam<bool>(control_nh, "use_gaussian", use_gaussian_, true);
  getParam<double>(control_nh, "gimbal_roll_delta_angle", gimbal_roll_delta_angle_, 0.1);
  getParam<double>(control_nh, "gimbal_pitch_delta_angle", gimbal_pitch_delta_angle_, 0.1);
  getParam<double>(control_nh, "gravity_acc", gravity_acc_, 1.0);
  getParam<double>(control_nh, "delay_param", delay_param_, 1.0);
  getParam<double>(control_nh, "gimbal_round_range", gimbal_round_range_, 0.2);
  getParam<double>(control_nh, "gaussian_exp", gaussian_exp_, 100);
}

void TwinHammerController::GimbalStatesCallback(sensor_msgs::JointState msg)
{
  for(int i=0; i<gimbal_states_angles_.size(); i++){
    gimbal_states_angles_.at(i) = msg.position.at(i);
  }
}

void TwinHammerController::HapticsSwitchCallback(std_msgs::Int8 msg)
{
  int i = msg.data;
  if(i==1){haptics_switch_ = true;}
  if(i==0){
    haptics_switch_ = false;
    haptics_force_ = Eigen::Vector3d::Zero();
    haptics_torque_ = Eigen::Vector3d::Zero();
  }
  // std::cout << "switch" << haptics_switch_ << std::endl;
}

void TwinHammerController::HapticsWrenchCallback(geometry_msgs::WrenchStamped msg)
{
  haptics_force_(0) = msg.wrench.force.x;
  haptics_force_(1) = msg.wrench.force.y;
  haptics_force_(2) = msg.wrench.force.z;
  haptics_torque_(0) = msg.wrench.torque.x;
  haptics_torque_(1) = msg.wrench.torque.y;
  haptics_torque_(2) = msg.wrench.torque.z;
  // std::cout << "wrench_x" << haptics_wrench_(0) << std::endl;
}

void TwinHammerController::controlCore()
{
  PoseLinearController::controlCore();
  tf::Vector3 target_acc_w(0.0, 0.0, 0.0);
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  if(use_haptics_flag_)
  {
    tf::Vector3 tf_haptics_force(haptics_force_.x(), haptics_force_.y(), haptics_force_.z()+gravity_acc_);
    target_acc_w = tf_haptics_force;
  }
  else
  {
    tf::Vector3 pid_result(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
    target_acc_w = pid_result;
  }
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  // Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);  
  // target_wrench_acc_cog_.head(3) = Eigen::Vector3d(target_acc_cog.x(),target_acc_cog.y(),target_acc_cog.z());
  target_wrench_acc_cog_.head(3) = Eigen::Vector3d(target_acc_w.x(),target_acc_w.y(),target_acc_w.z());

  double target_ang_acc_x = 0.0;
  double target_ang_acc_y = 0.0;
  double target_ang_acc_z = 0.0;

  if(use_haptics_flag_)
  {
    target_ang_acc_x = haptics_torque_.x();
    target_ang_acc_y = haptics_torque_.y();
    target_ang_acc_z = haptics_torque_.z();
  }
  else
  {
    target_ang_acc_x = pid_controllers_.at(ROLL).result();
    target_ang_acc_y = pid_controllers_.at(PITCH).result();
    target_ang_acc_z = pid_controllers_.at(YAW).result();
  }
  target_wrench_acc_cog_.tail(3) = Eigen::Vector3d(target_ang_acc_x,target_ang_acc_y,target_ang_acc_z);

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  double virtual_rotor_num = motor_num_/2;
  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3*virtual_rotor_num);
  double mass_inv = 1/twin_hammer_model_->getMass();
  Eigen::Matrix3d inertia = twin_hammer_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Matrix3d inertia_inv = (twin_hammer_model_->getInertia<Eigen::Matrix3d>()).inverse();
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = twin_hammer_model_->getRotorsOriginFromCog<Eigen::Vector3d>();

  Eigen::Vector3d virtual_rotor_1_origin(0.0,0.0,0.0);
  Eigen::Vector3d virtual_rotor_2_origin(0.0,0.0,0.0);
  for(int i=0; i<3; i++){
    virtual_rotor_1_origin(i) = (rotors_origin_from_cog.at(0)(i) + rotors_origin_from_cog.at(2)(i)) / 2;
    virtual_rotor_2_origin(i) = (rotors_origin_from_cog.at(1)(i) + rotors_origin_from_cog.at(3)(i)) / 2;
  }
  std::vector<Eigen::Vector3d> virtual_rotors_origin_from_cog = {virtual_rotor_1_origin, virtual_rotor_2_origin};

  double t = ros::Time::now().toSec();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6,3);
  wrench_map.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);
  int last_col = 0;
  for(int i=0; i<virtual_rotor_num; i++)
  {
    Eigen::Matrix3d skew_rotor_mat = aerial_robot_model::skew(virtual_rotors_origin_from_cog.at(i));
    wrench_map.block(3,0,3,3) = aerial_robot_model::skew(virtual_rotors_origin_from_cog.at(i));
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }
  // full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  // full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  Eigen::MatrixXd q1_mat = Eigen::MatrixXd::Zero(5,3*virtual_rotor_num); // remove tx
  q1_mat.topRows(3) = full_q_mat.topRows(3);
  q1_mat.bottomRows(2) = full_q_mat.bottomRows(2);
  Eigen::MatrixXd q1_mat_inv = aerial_robot_model::pseudoinverse(q1_mat);
  Eigen::VectorXd target_wrench_acc_cog_5d = Eigen::VectorXd::Zero(5); // remove tx
  target_wrench_acc_cog_5d.head(3) = target_wrench_acc_cog_.head(3);
  target_wrench_acc_cog_5d.tail(2) = target_wrench_acc_cog_.tail(2);
  target_vectoring_f_ = q1_mat_inv * target_wrench_acc_cog_5d;

  last_col = 0;
  double virtual_thrust_1 = 0;
  double virtual_thrust_2 = 0;
  for(int i=0; i<virtual_rotor_num; i++)
  {
    Eigen::Vector3d f_i = target_vectoring_f_.segment(last_col,3);
    if(i==0){virtual_thrust_1 = f_i.norm();}
    if(i==1){virtual_thrust_2 = f_i.norm();}
    double gimbal_i_roll = atan2(-f_i.y(), f_i.z());
    if(gimbal_i_roll > 3.1 || gimbal_i_roll < -3.1){gimbal_i_roll = 0.0;}
    double gimbal_i_pitch = atan2(f_i.x(), -f_i.y() * sin(gimbal_i_roll) + f_i.z() * cos(gimbal_i_roll));
    if(gimbal_i_pitch > 3.1 || gimbal_i_pitch < -3.1){gimbal_i_pitch = 3.14159265;}
    // if(i==0){std::cout << gimbal_i_roll << std::endl;}
    // if(gimbal_i_roll > prev_gimbal_angles_.at(2*i) + gimbal_roll_delta_angle_){
    //   gimbal_i_roll = prev_gimbal_angles_.at(2*i) + gimbal_roll_delta_angle_;
    // }
    // if(gimbal_i_roll < prev_gimbal_angles_.at(2*i) - gimbal_roll_delta_angle_){
    //   gimbal_i_roll = prev_gimbal_angles_.at(2*i) - gimbal_roll_delta_angle_;
    // }
    // if(gimbal_i_pitch > prev_gimbal_angles_.at(2*i+1) + gimbal_pitch_delta_angle_){
    //   gimbal_i_pitch = prev_gimbal_angles_.at(2*i+1) + gimbal_pitch_delta_angle_;
    // }
    // if(gimbal_i_pitch < prev_gimbal_angles_.at(2*i+1) - gimbal_pitch_delta_angle_){
    //   gimbal_i_pitch = prev_gimbal_angles_.at(2*i+1) - gimbal_pitch_delta_angle_;
    // }
    if(i==0){
      filtered_gimbal_1_roll_ = (1-delay_param_) * filtered_gimbal_1_roll_ + delay_param_ * gimbal_i_roll;
      double diff_gimbal_1_roll = gimbal_i_roll - gimbal_states_angles_.at(0);
      if(abs(diff_gimbal_1_roll) < gimbal_round_range_){
        if(use_polynominal_){
          double poly_rounded_gimbal_1_roll = GimbalRoundPolynominal(diff_gimbal_1_roll, gimbal_round_range_);
          target_gimbal_angles_.at(0) = poly_rounded_gimbal_1_roll;
        }
        else if(use_gaussian_){
          double gaussian_rounded_gimbal_1_roll = GimbalRoundGaussian(diff_gimbal_1_roll, gimbal_round_range_, gaussian_exp_);
          target_gimbal_angles_.at(0) = gaussian_rounded_gimbal_1_roll;
        }
        else{
          target_gimbal_angles_.at(0) = filtered_gimbal_1_roll_;
        }
      }
      else{
        target_gimbal_angles_.at(0) = filtered_gimbal_1_roll_;
      }
    }
    if(i==1){
      filtered_gimbal_2_roll_ = (1-delay_param_) * filtered_gimbal_2_roll_ + delay_param_ * gimbal_i_roll;
      double diff_gimbal_2_roll = gimbal_i_roll + gimbal_states_angles_.at(2);
      if(abs(diff_gimbal_2_roll) < gimbal_round_range_){
        if(use_polynominal_){
          double poly_rounded_gimbal_2_roll = GimbalRoundPolynominal(diff_gimbal_2_roll, gimbal_round_range_);
          target_gimbal_angles_.at(2) = poly_rounded_gimbal_2_roll;
        }
        else if(use_gaussian_){
          double gaussian_rounded_gimbal_2_roll = GimbalRoundGaussian(diff_gimbal_2_roll, gimbal_round_range_, gaussian_exp_);
          target_gimbal_angles_.at(2) = gaussian_rounded_gimbal_2_roll;
        }
        else{
          target_gimbal_angles_.at(2) = filtered_gimbal_2_roll_;
        }
      }
      else{
        target_gimbal_angles_.at(2) = filtered_gimbal_2_roll_;
      }
    }

    double rounded_gimbal_i_roll = GimbalRoundPolynominal(gimbal_i_roll, gimbal_round_range_);

    // target_gimbal_angles_.at(2*i) = gimbal_i_roll;
    target_gimbal_angles_.at(2*i+1) = gimbal_i_pitch;
    last_col += 3;
  }
  for(int i=0; i<prev_gimbal_angles_.size(); i++){
    prev_gimbal_angles_.at(i) = target_gimbal_angles_.at(i);
  }

  Eigen::Vector3d target_vec = Eigen::Vector3d::Zero(3);
  double t_x = target_wrench_acc_cog_(3);
  target_vec(0) = virtual_thrust_1;
  target_vec(1) = virtual_thrust_2;
  target_vec(2) = t_x;

  Eigen::MatrixXd q2_mat = Eigen::MatrixXd::Zero(3,motor_num_);
  for(int i=0; i<motor_num_; i++)
  {
    double pitch_angle = 0;
    if(i%2 == 0){
      q2_mat(0,i) = 1;
      pitch_angle = target_gimbal_angles_.at(1);
    }
    if(i%2 == 1){
      q2_mat(1,i) = 1;
      pitch_angle = target_gimbal_angles_.at(3);
    }
    double rotor_moment_arm = abs(rotors_origin_from_cog.at(i)(1)) * cos(pitch_angle);
    // if(rotor_moment_arm < 0.12){
    //   rotor_moment_arm = 0.0;
    // }
    if(i==0 || i==1){
      q2_mat(2,i) = rotor_moment_arm;
    }
    if(i==2 || i==3){
      q2_mat(2,i) = -rotor_moment_arm;
    }
  }
  Eigen::MatrixXd q2_mat_inv = aerial_robot_model::pseudoinverse(q2_mat);
  // for(int i=0; i<q2_mat.rows(); i++){
  //   for(int j=0; j<q2_mat.cols(); j++){
  //     std::cout << q2_mat(i,j) << ",";
  //    }
  //   std::cout << std::endl;
  // }
  // std::cout << "-----------" << std::endl;

  Eigen::VectorXd target_thrusts = q2_mat_inv * target_vec;
  for(int i=0; i<motor_num_; i++){
    if(target_thrusts(i)<0){
      ROS_WARN_STREAM("thrust at rotor " << i << " is minus value");
      target_thrusts(i) = 0.0;
    }
    target_base_thrust_.at(i) = target_thrusts(i);
  }
}

void TwinHammerController::sendCmd()
{
  PoseLinearController::sendCmd();
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);

  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for(int i=0; i<motor_num_; i++)
  {
    if(i<2){gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));}
    else{gimbal_control_msg.position.push_back(-target_gimbal_angles_.at(i));} // this is hard cording
  }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::TwinHammerController, aerial_robot_control::ControlBase);
