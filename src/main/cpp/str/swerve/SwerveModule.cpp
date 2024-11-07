// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/swerve/SwerveModule.h"

#include <utility>

#include "ctre/phoenix/StatusCodes.h"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/Alert.h"
#include "frc/DataLogManager.h"
#include "units/angle.h"

using namespace str::swerve;

SwerveModule::SwerveModule(const ModuleConstants& consts,
                           const ModulePhysicalCharacteristics& physical,
                           SteerGains steer, DriveGains drive)
    : moduleNamePrefix{consts.moduleName},
      encoderAlertMsg{moduleNamePrefix + " Steer Encoder Configuration"},
      steerAlertMsg{moduleNamePrefix + " Steer Motor Configuration"},
      driveAlertMsg{moduleNamePrefix + " Drive Motor Configuration"},
      configureEncoderAlert(encoderAlertMsg, frc::Alert::AlertType::kError),
      configureSteerAlert(steerAlertMsg, frc::Alert::AlertType::kError),
      configureDriveAlert(driveAlertMsg, frc::Alert::AlertType::kError),
      steerGains{std::move(steer)},
      driveGains{std::move(drive)},
      steerEncoder{consts.encoderId, "*"},
      steerMotor{consts.steerId, "*"},
      driveMotor{consts.driveId, "*"} {
  ConfigureSteerEncoder(consts.steerEncoderOffset);
  ConfigureSteerMotor();
  ConfigureDriveMotor();
}

void SwerveModule::ConfigureSteerEncoder(units::turn_t encoderOffset) {
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};

  encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
  encoderConfig.MagnetSensor.AbsoluteSensorRange =
      ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
  encoderConfig.MagnetSensor.SensorDirection =
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;

  ctre::phoenix::StatusCode configResult =
      steerEncoder.GetConfigurator().Apply(encoderConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured steer encoder on {} module. Result was: {}\n",
                  moduleNamePrefix, configResult.GetName()));

  configureEncoderAlert.Set(!configResult.IsOK());
}

void SwerveModule::ConfigureSteerMotor() {
  configureSteerAlert.Set(true);
}

void SwerveModule::ConfigureDriveMotor() {
  configureDriveAlert.Set(true);
}
