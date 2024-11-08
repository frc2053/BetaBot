// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/Alert.h>
#include <units/angle.h>

#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "SwerveModuleHelpers.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "str/swerve/SwerveModuleSim.h"
#include "units/current.h"
#include "units/dimensionless.h"

namespace str::swerve {

class SwerveModule {
 public:
  explicit SwerveModule(const ModuleConstants& consts,
                        const ModulePhysicalCharacteristics& physical,
                        SteerGains steer, DriveGains drive);
  void OptimizeBusSignals();

 private:
  void ConfigureSteerEncoder(units::turn_t encoderOffset);
  void ConfigureSteerMotor(bool invert, units::scalar_t gearing,
                           units::ampere_t supplyLim,
                           units::ampere_t statorLim);
  void ConfigureDriveMotor(bool invert, units::ampere_t supplyLim,
                           units::ampere_t statorLim);
  void ConfigureControlSignals();

  std::string moduleNamePrefix;

  std::string encoderAlertMsg;
  std::string steerAlertMsg;
  std::string driveAlertMsg;
  std::string optimizeSteerMsg;
  std::string optimizeDriveMsg;
  frc::Alert configureEncoderAlert;
  frc::Alert configureSteerAlert;
  frc::Alert configureDriveAlert;
  frc::Alert optimizeSteerMotorAlert;
  frc::Alert optimizeDriveMotorAlert;

  SteerGains steerGains;
  DriveGains driveGains;

  ctre::phoenix6::hardware::CANcoder steerEncoder;
  ctre::phoenix6::hardware::TalonFX steerMotor;
  ctre::phoenix6::hardware::TalonFX driveMotor;

  ctre::phoenix6::StatusSignal<units::turn_t> steerPositionSig =
      steerMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> steerVelocitySig =
      steerMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> steerTorqueCurrentSig =
      steerMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> steerVoltageSig =
      steerMotor.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::turn_t> drivePositionSig =
      driveMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> driveVelocitySig =
      driveMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::ampere_t> driveTorqueCurrentSig =
      driveMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> driveVoltageSig =
      driveMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC steerAngleSetter{
      0_rad};
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC driveVelocitySetter{
      0_rad_per_s};

  ctre::phoenix6::controls::VoltageOut steerVoltageSetter{0_V};
  ctre::phoenix6::controls::VoltageOut driveVoltageSetter{0_V};

  SwerveModuleSim moduleSim;
};
}  // namespace str::swerve
