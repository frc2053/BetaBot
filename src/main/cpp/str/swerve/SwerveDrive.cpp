// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/swerve/SwerveDrive.h"

#include <frc/DataLogManager.h>

#include "constants/SwerveConstants.h"
#include "ctre/phoenix/StatusCodes.h"
#include "frc/Alert.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include <frc/DriverStation.h>

using namespace str::swerve;

SwerveDrive::SwerveDrive()
    : imuConfigAlert{imuConfigAlertStr, frc::Alert::AlertType::kError},
      imuOptimizeAlert{imuOptimizeAlertStr, frc::Alert::AlertType::kError} {
  ConfigureImu();
  SetupSignals();
}

frc::Pose2d SwerveDrive::GetPose() const {
  return poseEstimator.GetEstimatedPosition();
}

void SwerveDrive::SetXModuleForces(
    const std::array<units::newton_t, 4>& xForce) {
  xModuleForce = xForce;
}

void SwerveDrive::SetYModuleForces(
    const std::array<units::newton_t, 4>& yForce) {
  yModuleForce = yForce;
}

void SwerveDrive::UpdateOdom() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(
          2.0 / consts::swerve::ODOM_UPDATE_RATE, allSignals);

  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error updating swerve odom! Error was: {}", status.GetName()));
  }

  int i = 0;
  for (auto& mod : modules) {
    modulePositions[i] = mod.GetPosition();
    moduleStates[i] = mod.GetState();
    i++;
  }

  yawLatencyComped =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          imu.GetYaw(), imu.GetAngularVelocityZWorld());
  poseEstimator.Update(frc::Rotation2d{yawLatencyComped}, modulePositions);
  odom.Update(frc::Rotation2d{yawLatencyComped}, modulePositions);

  units::second_t now = frc::Timer::GetFPGATimestamp();
  odomUpdateRate = 1.0 / (now - lastOdomUpdateTime);
  lastOdomUpdateTime = now;
}

void SwerveDrive::UpdateSimulation() {
  std::array<frc::SwerveModuleState, 4> simState;
  int i = 0;
  for (auto& swerveModule : modules) {
    simState[i] = swerveModule.UpdateSimulatedModule(
        frc::RobotController::GetBatteryVoltage());
    i++;
  }

  simStatesPub.Set(simState);

  units::radians_per_second_t omega =
      consts::swerve::KINEMATICS.ToChassisSpeeds(simState).omega;
  units::radian_t angleChange = omega * (1 / 50_Hz);

  lastSimAngle = lastSimAngle + frc::Rotation2d{angleChange};
  imuSimState.SetRawYaw(lastSimAngle.Degrees());
  imuSimState.SetAngularVelocityZ(omega);
}

void SwerveDrive::UpdateNTEntries() {
  odomUpdateRatePub.Set(odomUpdateRate.value());
}

void SwerveDrive::SetupSignals() {
  // If you are changing this you're prob cooked ngl
  for (size_t i = 0; i < modules.size(); i++) {
    const auto& modSigs = modules[i].GetSignals();
    allSignals[(i * 8) + 0] = modSigs[0];
    allSignals[(i * 8) + 1] = modSigs[1];
    allSignals[(i * 8) + 2] = modSigs[2];
    allSignals[(i * 8) + 3] = modSigs[3];
    allSignals[(i * 8) + 4] = modSigs[4];
    allSignals[(i * 8) + 5] = modSigs[5];
    allSignals[(i * 8) + 6] = modSigs[6];
    allSignals[(i * 8) + 7] = modSigs[7];
  }

  allSignals[allSignals.size() - 2] = &imu.GetYaw();
  allSignals[allSignals.size() - 1] = &imu.GetAngularVelocityZWorld();

  for (const auto& sig : allSignals) {
    sig->SetUpdateFrequency(consts::swerve::ODOM_UPDATE_RATE);
  }

  for (auto& mod : modules) {
    mod.OptimizeBusSignals();
  }

  ctre::phoenix::StatusCode optimizeImuResult = imu.OptimizeBusUtilization();

  frc::DataLogManager::Log(
      fmt::format("Optimized bus signals for imu. Result was: {}",
                  optimizeImuResult.GetName()));

  if (!optimizeImuResult.IsOK()) {
    imuOptimizeAlert.Set(true);
  }
}

void SwerveDrive::ConfigureImu() {
  ctre::phoenix6::configs::Pigeon2Configuration imuConfig;
  imuConfig.MountPose.MountPoseRoll = consts::swerve::IMU_MOUNT_ROLL;
  imuConfig.MountPose.MountPosePitch = consts::swerve::IMU_MOUNT_PITCH;
  imuConfig.MountPose.MountPoseYaw = consts::swerve::IMU_MOUNT_YAW;

  ctre::phoenix::StatusCode imuConfigStatus =
      imu.GetConfigurator().Apply(imuConfig);

  frc::DataLogManager::Log(
      fmt::format("Imu Configured. Result was: {}", imuConfigStatus.GetName()));

  if (!imuConfigStatus.IsOK()) {
    imuConfigAlert.Set(true);
  }
}

void SwerveDrive::DriveFieldRelative(units::meters_per_second_t xVel,
                                     units::meters_per_second_t yVel,
                                     units::radians_per_second_t omega,
                                     bool openLoop) {
  frc::ChassisSpeeds speedsToSend{};

  frc::Rotation2d rot = poseEstimator.GetEstimatedPosition().Rotation();

  if (frc::DriverStation::IsTeleop()) {
    rot = odom.GetPose().Rotation();
  }

  speedsToSend =
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, omega, rot);

  Drive(speedsToSend.vx, speedsToSend.vx, speedsToSend.omega, openLoop);
}

void SwerveDrive::Drive(units::meters_per_second_t xVel,
                        units::meters_per_second_t yVel,
                        units::radians_per_second_t omega, bool openLoop) {
  frc::ChassisSpeeds speedsToSend;
  speedsToSend.vx = xVel;
  speedsToSend.vy = yVel;
  speedsToSend.omega = omega;

  // https://github.com/wpilibsuite/allwpilib/issues/7332
  wpi::array<frc::SwerveModuleState, 4> tempStates =
      consts::swerve::KINEMATICS.ToSwerveModuleStates(speedsToSend);
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      &tempStates, consts::swerve::PHY_CHAR.MaxLinearSpeed());
  frc::ChassisSpeeds speeds =
      consts::swerve::KINEMATICS.ToChassisSpeeds(tempStates);

  speeds = frc::ChassisSpeeds::Discretize(speeds, (1 / 50_Hz));

  std::array<frc::SwerveModuleState, 4> states =
      consts::swerve::KINEMATICS.ToSwerveModuleStates(speeds);

  SetModuleStates(
      states, true, openLoop,
      ConvertModuleForcesToTorqueCurrent(xModuleForce, yModuleForce));
}

void SwerveDrive::SetModuleStates(
    const std::array<frc::SwerveModuleState, 4>& desiredStates, bool optimize,
    bool openLoop, const std::array<units::ampere_t, 4> moduleTorqueCurrentFF) {
  wpi::array<frc::SwerveModuleState, 4> finalState = desiredStates;
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      &finalState, consts::swerve::PHY_CHAR.MaxLinearSpeed());
  int i = 0;
  for (auto& mod : modules) {
    finalState[i] = mod.GoToState(finalState[i], optimize, openLoop,
                                  moduleTorqueCurrentFF[i]);
    i++;
  }
  // desiredStatesPub.Set(finalState);
}

std::array<units::ampere_t, 4> SwerveDrive::ConvertModuleForcesToTorqueCurrent(
    const std::array<units::newton_t, 4>& xForce,
    const std::array<units::newton_t, 4>& yForce) {
  std::array<frc::SwerveModuleState, 4> forces;

  std::array<units::ampere_t, 4> retVal;
  for (int i = 0; i < 4; i++) {
    if (xForce[i] == 0_N && yForce[0] == 0_N) {
      break;
    }
    frc::Translation2d moduleForceFieldRef{units::meter_t{xForce[i].value()},
                                           units::meter_t{yForce[i].value()}};
    frc::Translation2d moduleForceRobotRef =
        moduleForceFieldRef.RotateBy(GetPose().Rotation());
    units::newton_meter_t totalTorqueAtMotor =
        (units::newton_t{moduleForceRobotRef.Norm().value()} *
         consts::swerve::PHY_CHAR.wheelRadius) /
        consts::swerve::PHY_CHAR.driveGearing;
    units::ampere_t expectedTorqueCurrent =
        totalTorqueAtMotor / consts::swerve::PHY_CHAR.driveMotor.Kt;
    retVal[i] = expectedTorqueCurrent;
    forces[i].angle = moduleForceRobotRef.Angle();
    forces[i].speed = units::feet_per_second_t{expectedTorqueCurrent.value()};
  }
  // forcesPub.Set(forces);

  return retVal;
}
