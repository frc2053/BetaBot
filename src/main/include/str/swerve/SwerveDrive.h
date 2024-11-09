// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructArrayTopic.h>

#include <memory>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "constants/SwerveConstants.h"
#include "frc/Alert.h"
#include "str/swerve/SwerveModule.h"

namespace str::swerve {
class SwerveDrive {
 public:
  SwerveDrive();
  void UpdateOdom();
  void UpdateSimulation();
  void UpdateNTEntries();

 private:
  void SetupSignals();
  void ConfigureImu();

  std::array<SwerveModule, 4> modules{
      SwerveModule{consts::swerve::FL_MODULE, consts::swerve::PHY_CHAR,
                   consts::swerve::STEER_GAINS, consts::swerve::DRIVE_GAINS},
      SwerveModule{consts::swerve::FR_MODULE, consts::swerve::PHY_CHAR,
                   consts::swerve::STEER_GAINS, consts::swerve::DRIVE_GAINS},
      SwerveModule{consts::swerve::BL_MODULE, consts::swerve::PHY_CHAR,
                   consts::swerve::STEER_GAINS, consts::swerve::DRIVE_GAINS},
      SwerveModule{consts::swerve::BR_MODULE, consts::swerve::PHY_CHAR,
                   consts::swerve::STEER_GAINS, consts::swerve::DRIVE_GAINS}};

  ctre::phoenix6::hardware::Pigeon2 imu{consts::swerve::IMU_ID, "*"};
  ctre::phoenix6::sim::Pigeon2SimState& imuSimState = imu.GetSimState();

  std::array<ctre::phoenix6::BaseStatusSignal*, 34> allSignals;

  std::array<frc::SwerveModulePosition, 4> modulePositions;
  std::array<frc::SwerveModuleState, 4> moduleStates;
  units::radian_t yawLatencyComped{0_rad};
  units::second_t lastOdomUpdateTime{0_s};
  units::hertz_t odomUpdateRate{0_Hz};
  frc::Rotation2d lastSimAngle;

  frc::SwerveDriveOdometry<4> odom{consts::swerve::KINEMATICS,
                                   frc::Rotation2d{0_deg}, modulePositions};
  frc::SwerveDrivePoseEstimator<4> poseEstimator{
      consts::swerve::KINEMATICS, frc::Rotation2d{0_deg}, modulePositions,
      frc::Pose2d{}};

  static constexpr std::string_view imuConfigAlertStr = "Imu Configuration";
  static constexpr std::string_view imuOptimizeAlertStr = "Imu Optimization";
  frc::Alert imuConfigAlert;
  frc::Alert imuOptimizeAlert;

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Swerve")};
  nt::StructArrayPublisher<frc::SwerveModuleState> simStatesPub{
      nt->GetStructArrayTopic<frc::SwerveModuleState>("SimStates").Publish()};
  nt::DoublePublisher odomUpdateRatePub{
      nt->GetDoubleTopic("OdomUpdateRate").Publish()};
};
}  // namespace str::swerve
