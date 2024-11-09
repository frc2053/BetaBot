// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>

#include <memory>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "constants/SwerveConstants.h"
#include "frc/Alert.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "str/swerve/SwerveModule.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/velocity.h"

namespace str::swerve {
class SwerveDrive {
 public:
  SwerveDrive();
  frc::Pose2d GetPose() const;

  void SetXModuleForces(const std::array<units::newton_t, 4>& xForce);
  void SetYModuleForces(const std::array<units::newton_t, 4>& yForce);
  void UpdateOdom();
  void UpdateSimulation();
  void UpdateNTEntries();
  void DriveFieldRelative(units::meters_per_second_t xVel,
                          units::meters_per_second_t yVel,
                          units::radians_per_second_t omega, bool openLoop);
  void Drive(units::meters_per_second_t xVel, units::meters_per_second_t yVel,
             units::radians_per_second_t omega, bool openLoop);

 private:
  void SetupSignals();
  void ConfigureImu();
  void SetModuleStates(
      const std::array<frc::SwerveModuleState, 4>& desiredStates, bool optimize,
      bool openLoop,
      const std::array<units::ampere_t, 4> moduleTorqueCurrentFF);
  std::array<units::ampere_t, 4> ConvertModuleForcesToTorqueCurrent(
      const std::array<units::newton_t, 4>& xForce,
      const std::array<units::newton_t, 4>& yForce);

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
  std::array<units::newton_t, 4> xModuleForce{};
  std::array<units::newton_t, 4> yModuleForce{};

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
