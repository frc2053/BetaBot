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
  ConfigureSteerMotor(consts.invertSteer, physical.steerGearing,
                      physical.steerSupplySideLimit,
                      physical.steerStatorCurrentLimit);
  ConfigureDriveMotor(consts.invertDrive, physical.driveSupplySideLimit,
                      physical.driveStatorCurrentLimit);
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

void SwerveModule::ConfigureSteerMotor(bool invert, units::scalar_t gearing,
                                       units::ampere_t supplyLim,
                                       units::ampere_t statorLim) {
  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};

  steerSlotConfig.kA = steerGains.kA.value();
  steerSlotConfig.kV = steerGains.kV.value();
  steerSlotConfig.kS = steerGains.kS.value();
  steerSlotConfig.kP = steerGains.kP.value();
  steerSlotConfig.kI = steerGains.kI.value();
  steerSlotConfig.kD = steerGains.kD.value();
  steerConfig.Slot0 = steerSlotConfig;

  steerConfig.MotionMagic.MotionMagicCruiseVelocity =
      steerGains.motionMagicCruiseVel;
  steerConfig.MotionMagic.MotionMagicExpo_kV = steerGains.motionMagicExpoKv;
  steerConfig.MotionMagic.MotionMagicExpo_kA = steerGains.motionMagicExpoKa;

  steerConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.RotorToSensorRatio = gearing;
  steerConfig.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

  steerConfig.TorqueCurrent.PeakForwardTorqueCurrent = statorLim;
  steerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -statorLim;

  steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  steerConfig.CurrentLimits.SupplyCurrentLimit = supplyLim;

  ctre::phoenix::StatusCode configResult =
      steerMotor.GetConfigurator().Apply(steerConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured steer motor on {} module. Result was: {}\n",
                  moduleNamePrefix, configResult.GetName()));

  configureSteerAlert.Set(!configResult.IsOK());
}

void SwerveModule::ConfigureDriveMotor(bool invert, units::ampere_t supplyLim,
                                       units::ampere_t statorLim) {
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};

  driveSlotConfig.kV = driveGains.kV.value();
  driveSlotConfig.kA = driveGains.kA.value();
  driveSlotConfig.kS = driveGains.kS.value();
  driveSlotConfig.kP = driveGains.kP.value();
  driveSlotConfig.kI = driveGains.kI.value();
  driveSlotConfig.kD = driveGains.kD.value();
  driveConfig.Slot0 = driveSlotConfig;

  driveConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = statorLim;
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -statorLim;

  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.CurrentLimits.StatorCurrentLimit = statorLim;

  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit = supplyLim;

  ctre::phoenix::StatusCode configResult =
      driveMotor.GetConfigurator().Apply(driveConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured drive motor on {} module. Result was: {}\n",
                  moduleNamePrefix, configResult.GetName()));

  configureDriveAlert.Set(!configResult.IsOK());
}
