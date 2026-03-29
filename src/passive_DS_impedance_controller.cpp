// This code was derived from franka_example controllers and from the code written by Nadia Figueroa (UPenn)
// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license

#include <passive_DS_impedance_controller.h>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <pseudo_inversion.h>
#include <kinematics_utils.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/filters.h>

#include <cmath>
#include <memory>


namespace franka_passive_ds_impedance_controller {
// PassiveDS class with a damping matrix aligned with the desired velocity
PassiveDS::PassiveDS(const double& lam0, const double& lam1)
    : eigVal0(lam0), eigVal1(lam1) {
  damping_eigval.setZero();
  baseMat.setIdentity();
  Dmat.setIdentity();
  control_output.setZero();

  set_damping_eigval(lam0, lam1);
}

PassiveDS::~PassiveDS() = default;

void PassiveDS::set_damping_eigval(const double& lam0, const double& lam1) {
  if (lam0 <= 0.0 || lam1 <= 0.0) {
    std::cerr << "PassiveDS: damping eigenvalues must be positive." << std::endl;
    return;
  }

  eigVal0 = lam0;
  eigVal1 = lam1;

  damping_eigval.setZero();
  damping_eigval(0, 0) = eigVal0;
  damping_eigval(1, 1) = eigVal1;
  damping_eigval(2, 2) = eigVal1;
}

void PassiveDS::updateDampingMatrix(const Eigen::Vector3d& ref_vel) {
  constexpr double kMinVelocityNorm = 1e-6;

  if (ref_vel.norm() <= kMinVelocityNorm) {
    Dmat = Eigen::Matrix3d::Identity();
    return;
  }

  // Orthonormal basis whose first axis is aligned with the
  // reference velocity direction
  baseMat.setRandom();
  baseMat.col(0) = ref_vel.normalized();

  for (unsigned int i = 1; i < 3; ++i) {
    for (unsigned int j = 0; j < i; ++j) {
      baseMat.col(i) -= baseMat.col(j).dot(baseMat.col(i)) * baseMat.col(j);
    }
    baseMat.col(i).normalize();
  }

  Dmat = baseMat * damping_eigval * baseMat.transpose();
}

void PassiveDS::update(const Eigen::Vector3d& vel,
                       const Eigen::Vector3d& des_vel) {
  updateDampingMatrix(des_vel);

  // Passive-DS wrench
  control_output = -Dmat * vel + eigVal0 * des_vel;
}

Eigen::Vector3d PassiveDS::get_output() {
  return control_output;
}

bool PassiveDSImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                        ros::NodeHandle& node_handle) {
  // ROS subscribers
  sub_desired_twist_ = node_handle.subscribe(
      "/passiveDS/desired_twist", 20,
      &PassiveDSImpedanceController::desiredTwistCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_desired_damping_ = node_handle.subscribe(
      "/passiveDS/desired_damp_eigval", 1000,
      &PassiveDSImpedanceController::desiredDampingCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Required ROS parameters
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("PassiveDSImpedanceController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Franka hardware interfaces and joint handles
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }

  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PassiveDSImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Default controller state initialization
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  velocity_d_.setZero();

  dx_linear_des_.resize(3);
  dx_linear_msr_.resize(3);
  orient_error.resize(3);
  F_linear_des_.resize(3);
  F_angular_des_.resize(3);
  F_ee_des_.resize(6);

  update_impedance_params_ = false;

  // Cartesian impedance parameters
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_setpoint_ctrl_.setIdentity();
  cartesian_stiffness_grav_comp_.setIdentity();
  cartesian_damping_target_.setIdentity();

  std::vector<double> cartesian_stiffness_setpoint_ctrl_yaml;
  if (!node_handle.getParam("cartesian_stiffness_setpoint_ctrl",
                            cartesian_stiffness_setpoint_ctrl_yaml) ||
      cartesian_stiffness_setpoint_ctrl_yaml.size() != 6) {
    ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no "
        "cartesian_stiffness_setpoint_ctrl parameters provided, aborting controller init!");
    return false;
  }
  for (int i = 0; i < 6; i++) {
    cartesian_stiffness_setpoint_ctrl_(i, i) = cartesian_stiffness_setpoint_ctrl_yaml[i];
  }

  std::vector<double> cartesian_stiffness_grav_comp_yaml;
  if (!node_handle.getParam("cartesian_stiffness_grav_comp",
                            cartesian_stiffness_grav_comp_yaml) ||
      cartesian_stiffness_grav_comp_yaml.size() != 6) {
    ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no "
        "cartesian_stiffness_grav_comp parameters provided, aborting controller init!");
    return false;
  }
  for (int i = 0; i < 6; i++) {
    cartesian_stiffness_grav_comp_(i, i) = cartesian_stiffness_grav_comp_yaml[i];
  }

  cartesian_stiffness_mode_ = 1;
  if (!node_handle.getParam("cartesian_stiffness_mode", cartesian_stiffness_mode_)) {
    ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no cartesian_stiffness_mode parameters "
        "provided, aborting controller init!");
    return false;
  }

  for (int i = 0; i < 6; i++) {
    if (cartesian_stiffness_mode_ == 0) {
      cartesian_stiffness_target_(i, i) = cartesian_stiffness_grav_comp_(i, i);
    } else {
      cartesian_stiffness_target_(i, i) = cartesian_stiffness_setpoint_ctrl_(i, i);
    }

    // Critical damping approximation for the initial Cartesian impedance
    cartesian_damping_target_(i, i) = 2.0 * sqrt(cartesian_stiffness_target_(i, i));
  }


  // Passive-DS parameters
  damping_eigvals_yaml_.setZero();
  std::vector<double> damping_eigvals;
  if (node_handle.getParam("linear_damping_eigenvalues", damping_eigvals)) {
    if (damping_eigvals.size() != 3) {
      ROS_ERROR(
          "PassiveDSImpedanceController: Invalid or no linear_damping_eigenvalues "
          "parameters provided, aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 3; ++i) {
      damping_eigvals_yaml_[i] = damping_eigvals.at(i);
    }
    ROS_INFO_STREAM("Damping Matrix Eigenvalues (from YAML): " << damping_eigvals_yaml_);
  }

  damping_eigval0_ = damping_eigvals_yaml_(0);
  damping_eigval1_ = damping_eigvals_yaml_(1);
  passive_ds_controller = std::make_unique<PassiveDS>(100., 100.);
  passive_ds_controller->set_damping_eigval(damping_eigval0_, damping_eigval1_);

  ang_damping_eigvals_yaml_.setZero();
  std::vector<double> ang_damping_eigvals;
  if (node_handle.getParam("angular_damping_eigenvalues", damping_eigvals)) {
    if (damping_eigvals.size() != 3) {
      ROS_ERROR(
          "PassiveDSImpedanceController: Invalid or no angular_damping_eigenvalues "
          "parameters provided, aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 3; ++i) {
      ang_damping_eigvals_yaml_[i] = damping_eigvals.at(i);
    }
    ROS_INFO_STREAM("Angular Damping Matrix Eigenvalues (from YAML): "
                    << ang_damping_eigvals_yaml_);
  }

  ang_damping_eigval0_ = ang_damping_eigvals_yaml_(0);
  ang_damping_eigval1_ = ang_damping_eigvals_yaml_(1);
  ang_passive_ds_controller = std::make_unique<PassiveDS>(5., 5.);
  ang_passive_ds_controller->set_damping_eigval(
      ang_damping_eigval0_, ang_damping_eigval1_);

  // Nullspace parameters
  if (!node_handle.getParam("nullspace_stiffness", nullspace_stiffness_target_) ||
      nullspace_stiffness_target_ <= 0) {
    ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no nullspace_stiffness parameters "
        "provided, aborting controller init!");
    return false;
  }
  ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl
                                                  << nullspace_stiffness_target_);

  q_d_nullspace_.setZero();
  std::vector<double> q_nullspace;
  if (node_handle.getParam("q_nullspace", q_nullspace)) {
    q_d_nullspace_initialized_ = true;
    if (q_nullspace.size() != 7) {
      ROS_ERROR(
          "PassiveDSImpedanceController: Invalid or no q_nullspace parameters provided, "
          "aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 7; ++i) {
      q_d_nullspace_[i] = q_nullspace.at(i);
    }
    ROS_INFO_STREAM("Desired nullspace position (from YAML): " << std::endl
                                                               << q_d_nullspace_);
  }

  // Dynamic reconfigure
  dynamic_reconfigure_passive_ds_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() +
                      "dynamic_reconfigure_passive_ds_param_node");

  dynamic_server_passive_ds_param_ = std::make_unique<
      dynamic_reconfigure::Server<
          franka_passive_ds_impedance_controller::passive_ds_paramConfig>>(
      dynamic_reconfigure_passive_ds_param_node_);

  dynamic_server_passive_ds_param_->setCallback(
      boost::bind(&PassiveDSImpedanceController::passiveDSParamCallback, this, _1, _2));
  dynamic_server_passive_ds_param_->getConfigDefault(config_cfg);

  return true;
}

void PassiveDSImpedanceController::starting(const ros::Time& /*time*/) {
  // Initialize robot state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(initial_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  // Store the initial external torque estimate after gravity compensation
  tau_ext_initial_ = tau_measured - gravity;

  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // Initialize Cartesian pose references from the current EE pose
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // Initialize the nullspace reference if it was not provided from config
  if (!q_d_nullspace_initialized_) {
    q_d_nullspace_ = q_initial;
    q_d_nullspace_initialized_ = true;
    ROS_INFO_STREAM("Desired nullspace position (from q_initial): "
                    << std::endl
                    << q_d_nullspace_);
  }

  // Reset controller state
  elapsed_time = ros::Duration(0.0);
  last_cmd_time = 0.0;
  last_msg_time = 0.0;
  vel_cmd_timeout = 0.25;

  real_damping_eigval0_ = damping_eigval0_;
  real_damping_eigval1_ = damping_eigval1_;
  desired_damp_eigval_cb_ = real_damping_eigval0_;
  desired_damp_eigval_cb_prev_ = real_damping_eigval0_;
  new_damping_msg_ = false;
}


void PassiveDSImpedanceController::update(const ros::Time& /*time*/,
                                          const ros::Duration& period) {
  // Read robot state
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext_hat(robot_state.O_F_ext_hat_K.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  F_ext_hat_ << F_ext_hat;

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // Compute task-space velocity
  Eigen::Matrix<double, 6, 1> velocity;
  Eigen::Matrix<double, 6, 1> velocity_desired_;

  velocity << jacobian * dq;
  velocity_desired_.setZero();
  velocity_desired_.head(3) << velocity_d_;

  elapsed_time += period;

  // Reset commanded velocity or damping override if the corresponding command has timed out
  if (ros::Time::now().toSec() - last_cmd_time > vel_cmd_timeout) {
    velocity_d_.setZero();
  }

  if (ros::Time::now().toSec() - last_msg_time > vel_cmd_timeout) {
    new_damping_msg_ = false;
  }

  // Compute desired Cartesian wrench from the Passive-DS controller
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_nullspace_error(7), tau_d(7);
  Eigen::VectorXd F_ee_des_;
  F_ee_des_.resize(6);

  dx_linear_des_ << velocity_d_;
  dx_linear_msr_ << velocity.head(3);
  dx_angular_msr_ << velocity.tail(3);

  // Translational Passive-DS block
  F_linear_des_.setZero();

  real_damping_eigval0_ = damping_eigval0_;
  real_damping_eigval1_ = damping_eigval1_;

  if (new_damping_msg_) {
    real_damping_eigval0_ = desired_damp_eigval_cb_;
    real_damping_eigval1_ = desired_damp_eigval_cb_;
  }

  if (velocity_d_.norm() < 0.00001) {
    real_damping_eigval0_ = 0.1;
    real_damping_eigval1_ = 0.1;
  }

  passive_ds_controller->set_damping_eigval(real_damping_eigval0_, real_damping_eigval1_);
  passive_ds_controller->update(dx_linear_msr_, dx_linear_des_);
  F_linear_des_ << passive_ds_controller->get_output();
  F_ee_des_.head(3) = F_linear_des_;

  desired_damp_eigval_cb_prev_ = desired_damp_eigval_cb_;

  // Generate a desired angular velocity from the current and target orientations,
  // then regulate angular motion through the angular Passive-DS block
  Eigen::Vector4d _ee_quat;
  _ee_quat.setZero();
  _ee_quat[0] = orientation.w();
  _ee_quat.segment(1, 3) = orientation.vec();

  Eigen::Vector4d _ee_des_quat;
  _ee_des_quat.setZero();
  _ee_des_quat[0] = orientation_d_.w();
  _ee_des_quat.segment(1, 3) = orientation_d_.vec();

  Eigen::Vector4d dqd =
      KinematicsUtils<double>::slerpQuaternion(_ee_quat, _ee_des_quat, 0.5);
  Eigen::Vector4d deltaQ = dqd - _ee_quat;
  Eigen::Vector4d qconj = _ee_quat;
  qconj.segment(1, 3) = -1 * qconj.segment(1, 3);

  Eigen::Vector4d temp_angVel =
      KinematicsUtils<double>::quaternionProduct(deltaQ, qconj);
  Eigen::Vector3d tmp_angular_vel = temp_angVel.segment(1, 3);

  double maxDq(0.3), dsGain_ori(10.0);
  if (tmp_angular_vel.norm() > maxDq) {
    tmp_angular_vel = maxDq * tmp_angular_vel.normalized();
  }

  double theta_gq =
      (-.5 / (4 * maxDq * maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
  dx_angular_des_ = 2 * dsGain_ori * (1 + std::exp(theta_gq)) * tmp_angular_vel;

  ang_passive_ds_controller->update(dx_angular_msr_, dx_angular_des_);
  F_angular_des_ << ang_passive_ds_controller->get_output();
  F_ee_des_.tail(3) = F_angular_des_;

  // Map desired Cartesian wrench to joint-space task torque
  tau_task << jacobian.transpose() * F_ee_des_;

  // nullspace regulation
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  Eigen::VectorXd nullspace_stiffness_vec(7);

  // Task-specific nullspace stiffness profile
  nullspace_stiffness_vec << 0.0001 * nullspace_stiffness_,
                              0.1 * nullspace_stiffness_,
                              5 * nullspace_stiffness_,
                              0.0001 * nullspace_stiffness_,
                              0.0001 * nullspace_stiffness_,
                              0.0001 * nullspace_stiffness_,
                              0.0001 * nullspace_stiffness_;

  for (int i = 0; i < 7; i++) {
    tau_nullspace_error(i) = nullspace_stiffness_vec(i) * (q_d_nullspace_(i) - q(i));
  }

  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (tau_nullspace_error - (2.0 * sqrt(nullspace_stiffness_)) * dq);


  tau_d << tau_task + tau_nullspace + coriolis;

  // Send saturated torques
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // Update runtime controller parameters and filter target pose commands to avoid discontinuous reference changes.
  cartesian_stiffness_ = cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
  nullspace_stiffness_ = nullspace_stiffness_target_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> PassiveDSImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// Passive-DS parameter callback
void PassiveDSImpedanceController::passiveDSParamCallback(
    franka_passive_ds_impedance_controller::passive_ds_paramConfig& config,
    uint32_t /*level*/) {

  config_cfg    = config;
  bPassiveOrient_             = config.activate_angular_passiveDS;  
  update_impedance_params_    = config.update_impedance_params;

  if (update_impedance_params_){
      damping_eigval0_ = config.damping_eigval0;
      damping_eigval1_ = config.damping_eigval1;
      passive_ds_controller->set_damping_eigval(damping_eigval0_,damping_eigval1_);

  }
}

// Desired twist callback from the DS
void PassiveDSImpedanceController::desiredTwistCallback(
    const geometry_msgs::TwistConstPtr& msg) {

  velocity_d_      << msg->linear.x, msg->linear.y, msg->linear.z;
  last_cmd_time    = ros::Time::now().toSec();

  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());

  double dt_call = 1./1000;
  double int_gain = 200;    
  position_d_target_ << position + velocity_d_*dt_call*int_gain;

}

// Desired damping callback
void PassiveDSImpedanceController::desiredDampingCallback(
    const std_msgs::Float32Ptr& msg) {
    
    desired_damp_eigval_cb_ =  msg->data;
    last_msg_time    = ros::Time::now().toSec();
    new_damping_msg_ = true;
}



}  // namespace franka_passive_ds_impedance_controller

PLUGINLIB_EXPORT_CLASS(franka_passive_ds_impedance_controller::PassiveDSImpedanceController,
                       controller_interface::ControllerBase)
