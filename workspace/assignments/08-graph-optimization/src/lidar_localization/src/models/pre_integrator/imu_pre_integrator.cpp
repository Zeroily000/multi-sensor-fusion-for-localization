/*
 * @Description: IMU pre-integrator for LIO mapping, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#include "lidar_localization/models/pre_integrator/imu_pre_integrator.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "glog/logging.h"

namespace lidar_localization {

IMUPreIntegrator::IMUPreIntegrator(const YAML::Node& node) {
    //
    // parse config:
    // 
    // a. earth constants:
    EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
    // b. process noise:
    COV.MEASUREMENT.ACCEL = node["covariance"]["measurement"]["accel"].as<double>();
    COV.MEASUREMENT.GYRO = node["covariance"]["measurement"]["gyro"].as<double>();
    COV.RANDOM_WALK.ACCEL = node["covariance"]["random_walk"]["accel"].as<double>();
    COV.RANDOM_WALK.GYRO = node["covariance"]["random_walk"]["gyro"].as<double>();    

    // prompt:
    LOG(INFO) << std::endl 
              << "IMU Pre-Integration params:" << std::endl
              << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
              << std::endl
              << "\tprocess noise:" << std::endl
              << "\t\tmeasurement:" << std::endl
              << "\t\t\taccel.: " << COV.MEASUREMENT.ACCEL << std::endl
              << "\t\t\tgyro.: " << COV.MEASUREMENT.GYRO << std::endl
              << "\t\trandom_walk:" << std::endl
              << "\t\t\taccel.: " << COV.RANDOM_WALK.ACCEL << std::endl
              << "\t\t\tgyro.: " << COV.RANDOM_WALK.GYRO << std::endl
              << std::endl;

    // a. gravity constant:
    state.g_ = Eigen::Vector3d(
        0.0, 
        0.0, 
        EARTH.GRAVITY_MAGNITUDE
    );

    // b. process noise:
    Q_.block<3, 3>(INDEX_M_ACC_PREV, INDEX_M_ACC_PREV) = Q_.block<3, 3>(INDEX_M_ACC_CURR, INDEX_M_ACC_CURR) = COV.MEASUREMENT.ACCEL * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(INDEX_M_GYR_PREV, INDEX_M_GYR_PREV) = Q_.block<3, 3>(INDEX_M_GYR_CURR, INDEX_M_GYR_CURR) = COV.MEASUREMENT.GYRO * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(INDEX_R_ACC_PREV, INDEX_R_ACC_PREV) = COV.RANDOM_WALK.ACCEL * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(INDEX_R_GYR_PREV, INDEX_R_GYR_PREV) = COV.RANDOM_WALK.GYRO * Eigen::Matrix3d::Identity();
}

/**
 * @brief  reset IMU pre-integrator
 * @param  init_imu_data, init IMU measurements
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::Init(const IMUData &init_imu_data) {
    // reset pre-integrator state:
    ResetState(init_imu_data);
    
    // mark as inited:
    is_inited_ = true;

    return true;
}

/**
 * @brief  update IMU pre-integrator
 * @param  imu_data, current IMU measurements
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::Update(const IMUData &imu_data) {
    if ( imu_data_buff_.front().time < imu_data.time ) {
        // set buffer:
        imu_data_buff_.push_back(imu_data);

        // update state mean, covariance and Jacobian:
        UpdateState();

        // move forward:
        imu_data_buff_.pop_front();
    }

    return true;
}

/**
 * @brief  reset IMU pre-integrator using new init IMU measurement
 * @param  init_imu_data, new init IMU measurements
 * @param  output pre-integration result for constraint building as IMUPreIntegration
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::Reset(
    const IMUData &init_imu_data, 
    IMUPreIntegration &imu_pre_integration
) {
    // one last update:
    Update(init_imu_data);

    // set output IMU pre-integration:
    imu_pre_integration.T_ = init_imu_data.time - time_;

    // set gravity constant:
    imu_pre_integration.g_ = state.g_;

    // set measurement:
    imu_pre_integration.alpha_ij_ = state.alpha_ij_;
    imu_pre_integration.theta_ij_ = state.theta_ij_;
    imu_pre_integration.beta_ij_ = state.beta_ij_;
    imu_pre_integration.b_a_i_ = state.b_a_i_;
    imu_pre_integration.b_g_i_ = state.b_g_i_;
    // set information:
    imu_pre_integration.P_ = P_;
    // set Jacobian:
    imu_pre_integration.J_ = J_;

    // reset:
    ResetState(init_imu_data);

    return true;
}

/**
 * @brief  reset pre-integrator state using IMU measurements
 * @param  void
 * @return void
 */
void IMUPreIntegrator::ResetState(const IMUData &init_imu_data) {
    // reset time:
    time_ = init_imu_data.time;

    // a. reset relative translation:
    state.alpha_ij_ = Eigen::Vector3d::Zero();
    // b. reset relative orientation:
    state.theta_ij_ = Sophus::SO3d();
    // c. reset relative velocity:
    state.beta_ij_ = Eigen::Vector3d::Zero();
    // d. set init bias, acceleometer:
    state.b_a_i_ = Eigen::Vector3d(
        init_imu_data.accel_bias.x,
        init_imu_data.accel_bias.y,
        init_imu_data.accel_bias.z
    );
    // d. set init bias, gyroscope:
    state.b_g_i_ = Eigen::Vector3d(
        init_imu_data.gyro_bias.x,
        init_imu_data.gyro_bias.y,
        init_imu_data.gyro_bias.z
    );

    // reset state covariance:
    P_ = MatrixP::Zero();

    // reset Jacobian:
    J_ = MatrixJ::Identity();

    // reset buffer:
    imu_data_buff_.clear();
    imu_data_buff_.push_back(init_imu_data);
}

/**
 * @brief  update pre-integrator state: mean, covariance and Jacobian
 * @param  void
 * @return void
 */
void IMUPreIntegrator::UpdateState() {
    //
    // parse measurements:
    //
    // get measurement handlers:
    IMUData const & prev_imu_data{imu_data_buff_.at(0)};
    IMUData const & curr_imu_data{imu_data_buff_.at(1)};

    // get time delta:
    double const dt{curr_imu_data.time - prev_imu_data.time};
    double const dt2{std::pow(dt, 2.0)};
    double const dt3{std::pow(dt, 3.0)};

    // get measurements:
    Eigen::Vector3d const prev_a{
            prev_imu_data.linear_acceleration.x - state.b_a_i_.x(),
            prev_imu_data.linear_acceleration.y - state.b_a_i_.y(),
            prev_imu_data.linear_acceleration.z - state.b_a_i_.z()
    };
    Eigen::Vector3d const curr_a{
            curr_imu_data.linear_acceleration.x - state.b_a_i_.x(),
            curr_imu_data.linear_acceleration.y - state.b_a_i_.y(),
            curr_imu_data.linear_acceleration.z - state.b_a_i_.z()
    };

    Eigen::Vector3d const prev_w{
            prev_imu_data.angular_velocity.x - state.b_g_i_.x(),
            prev_imu_data.angular_velocity.y - state.b_g_i_.y(),
            prev_imu_data.angular_velocity.z - state.b_g_i_.z()
    };
    Eigen::Vector3d const curr_w{
            curr_imu_data.angular_velocity.x - state.b_g_i_.x(),
            curr_imu_data.angular_velocity.y - state.b_g_i_.y(),
            curr_imu_data.angular_velocity.z - state.b_g_i_.z()
    };

    //
    // TODO: a. update mean:
    //
    // 1. get w_mid:
    Eigen::Vector3d const w_mid{0.5*( prev_w + curr_w )};
    // 2. update relative orientation, so3:
    Sophus::SO3d const prev_theta_ij{state.theta_ij_};
    Sophus::SO3d const curr_theta_ij{state.theta_ij_*Sophus::SO3d::exp(w_mid*dt)};
    state.theta_ij_ = curr_theta_ij;
    // 3. get a_mid:
    Eigen::Vector3d const a_mid{0.5*(prev_theta_ij*prev_a + curr_theta_ij*curr_a)};
    // 4. update relative translation:
    state.alpha_ij_ += state.beta_ij_*dt + 0.5*a_mid*dt2;
    // 5. update relative velocity:
    state.beta_ij_ += a_mid*dt;

    //
    // TODO: b. update covariance:
    //
    // 1. intermediate results:
    Eigen::Matrix3d const dR_inv{Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(w_mid*dt)};
    Eigen::Matrix3d const prev_R{prev_theta_ij.matrix()};
    Eigen::Matrix3d const curr_R{curr_theta_ij.matrix()};
    Eigen::Matrix3d const prev_R_a_hat{prev_R*Sophus::SO3d::hat(prev_a)};
    Eigen::Matrix3d const curr_R_a_hat{curr_R*Sophus::SO3d::hat(curr_a)};

    //
    // TODO: 2. set up F:
    //
    MatrixF F{MatrixF::Zero()};
    F.block<3, 3>(INDEX_ALPHA, INDEX_ALPHA) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = -0.25*(prev_R_a_hat + curr_R_a_hat*dR_inv)*dt2;
    F.block<3, 3>(INDEX_ALPHA, INDEX_BETA) = Eigen::Matrix3d::Identity()*dt;
    F.block<3, 3>(INDEX_ALPHA, INDEX_B_A) = -0.25*(prev_R + curr_R)*dt2;
    F.block<3, 3>(INDEX_ALPHA, INDEX_B_G) = 0.25*(curr_R_a_hat)*dt3;

    F.block<3, 3>(INDEX_THETA, INDEX_THETA) = dR_inv;
    F.block<3, 3>(INDEX_THETA, INDEX_B_G) = -Eigen::Matrix3d::Identity()*dt;

    F.block<3, 3>(INDEX_BETA, INDEX_THETA) = -0.5*(prev_R_a_hat + curr_R_a_hat*dR_inv)*dt;
    F.block<3, 3>(INDEX_BETA, INDEX_BETA) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(INDEX_BETA, INDEX_B_A) = -0.5*(prev_R + curr_R)*dt;
    F.block<3, 3>(INDEX_BETA, INDEX_B_G) = 0.5*curr_R_a_hat*dt2;

    F.block<3, 3>(INDEX_B_A, INDEX_B_A) = Eigen::Matrix3d::Identity();

    F.block<3, 3>(INDEX_B_G, INDEX_B_G) = Eigen::Matrix3d::Identity();
//    MatrixF F = F_;

    //
    // TODO: 3. set up G:
    //
    MatrixB B{MatrixB::Zero()};
    B.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_PREV) = 0.25*prev_R*dt2;
    B.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_PREV) = -0.125*curr_R_a_hat*dt3;
    B.block<3, 3>(INDEX_ALPHA, INDEX_M_ACC_CURR) = 0.25*curr_R*dt2;
    B.block<3, 3>(INDEX_ALPHA, INDEX_M_GYR_CURR) = -0.125*curr_R_a_hat*dt3;

    B.block<3, 3>(INDEX_THETA, INDEX_M_GYR_CURR) = 0.5*Eigen::Matrix3d::Identity();
    B.block<3, 3>(INDEX_THETA, INDEX_M_GYR_PREV) = 0.5*Eigen::Matrix3d::Identity();

    B.block<3, 3>(INDEX_BETA, INDEX_M_ACC_PREV) = 0.5*prev_R;
    B.block<3, 3>(INDEX_BETA, INDEX_M_GYR_PREV) = -0.25*curr_R_a_hat*dt2;
    B.block<3, 3>(INDEX_BETA, INDEX_M_ACC_CURR) = 0.5*curr_R*dt;
    B.block<3, 3>(INDEX_BETA, INDEX_M_GYR_CURR) = -0.25*curr_R_a_hat*dt2;

    B.block<3, 3>(INDEX_B_A, INDEX_R_ACC_PREV) = Eigen::Matrix3d::Identity()*dt;

    B.block<3, 3>(INDEX_B_G, INDEX_R_GYR_PREV) = Eigen::Matrix3d::Identity()*dt;

    // TODO: 4. update P_:
    P_ = F*P_*F.transpose() + B*Q_*B.transpose();

    //
    // TODO: 5. update Jacobian:
    //
    J_ = F * J_;
}

} // namespace lidar_localization