// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

// #include <choreo/trajectory/SwerveSample.h>
#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"

using namespace ctre::phoenix6;

namespace subsystems {

class CommandSwerveDrivetrain : public frc2::SubsystemBase,
                                public swerve::SwerveDrivetrain {
  static constexpr units::second_t kSimLoopPeriod = 5_ms;
  std::unique_ptr<frc::Notifier> m_simNotifier;
  units::second_t m_lastSimTime;

  static constexpr frc::Rotation2d kBlueAlliancePerspectiveRotation{0_deg};
  static constexpr frc::Rotation2d kRedAlliancePerspectiveRotation{180_deg};
  bool m_hasAppliedOperatorPerspective = false;

  swerve::requests::ApplyFieldSpeeds m_pathApplyFieldSpeeds;
  frc::PIDController m_pathXController{10, 0, 0};
  frc::PIDController m_pathYController{10, 0, 0};
  frc::PIDController m_pathThetaController{7, 0, 0};

  swerve::requests::SysIdSwerveTranslation m_translationCharacterization;
  swerve::requests::SysIdSwerveSteerGains m_steerCharacterization;
  swerve::requests::SysIdSwerveRotation m_rotationCharacterization;

  frc2::sysid::SysIdRoutine m_sysIdRoutineTranslation{
      frc2::sysid::Config{
          std::nullopt, 4_V, std::nullopt,
          [](frc::sysid::State state) {
            SignalLogger::WriteString(
                "SysIdTranslation_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t output) {
            SetControl(m_translationCharacterization.WithVolts(output));
          },
          {},
          this}};

  frc2::sysid::SysIdRoutine m_sysIdRoutineSteer{
      frc2::sysid::Config{
          std::nullopt, 7_V, std::nullopt,
          [](frc::sysid::State state) {
            SignalLogger::WriteString(
                "SysIdSteer_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t output) {
                               SetControl(
                                   m_steerCharacterization.WithVolts(output));
                             },
                             {},
                             this}};

  frc2::sysid::SysIdRoutine m_sysIdRoutineRotation{
      frc2::sysid::Config{
          units::constants::detail::PI_VAL / 6 * (1_V / 1_s),
          units::constants::detail::PI_VAL * 1_V, std::nullopt,
          [](frc::sysid::State state) {
            SignalLogger::WriteString(
                "SysIdRotation_State",
                frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t output) {
            SetControl(m_rotationCharacterization.WithRotationalRate(
                output * (1_rad_per_s / 1_V)));
            SignalLogger::WriteValue("Rotational_Rate",
                                     output * (1_rad_per_s / 1_V));
          },
          {},
          this}};

  frc2::sysid::SysIdRoutine* m_sysIdRoutineToApply = &m_sysIdRoutineTranslation;

 public:
  template <std::same_as<swerve::SwerveModuleConstants>... ModuleConstants>
  CommandSwerveDrivetrain(
      swerve::SwerveDrivetrainConstants const& driveTrainConstants,
      ModuleConstants const&... modules)
      : SwerveDrivetrain{driveTrainConstants, modules...} {
    if (utils::IsSimulation()) {
      StartSimThread();
    }
  }

  template <std::same_as<swerve::SwerveModuleConstants>... ModuleConstants>
  CommandSwerveDrivetrain(
      swerve::SwerveDrivetrainConstants const& driveTrainConstants,
      units::hertz_t odometryUpdateFrequency, ModuleConstants const&... modules)
      : SwerveDrivetrain{driveTrainConstants, odometryUpdateFrequency,
                         modules...} {
    if (utils::IsSimulation()) {
      StartSimThread();
    }
  }

  template <std::same_as<swerve::SwerveModuleConstants>... ModuleConstants>
  CommandSwerveDrivetrain(
      swerve::SwerveDrivetrainConstants const& driveTrainConstants,
      units::hertz_t odometryUpdateFrequency,
      std::array<double, 3> const& odometryStandardDeviation,
      std::array<double, 3> const& visionStandardDeviation,
      ModuleConstants const&... modules)
      : SwerveDrivetrain{driveTrainConstants, odometryUpdateFrequency,
                         odometryStandardDeviation, visionStandardDeviation,
                         modules...} {
    if (utils::IsSimulation()) {
      StartSimThread();
    }
  }

  template <typename RequestSupplier>
    requires std::derived_from<
        std::remove_reference_t<std::invoke_result_t<RequestSupplier>>,
        swerve::requests::SwerveRequest>
  frc2::CommandPtr ApplyRequest(RequestSupplier request) {
    return Run(
        [this, request = std::move(request)] { return SetControl(request()); });
  }

  // void FollowPath(frc::Pose2d const &pose, choreo::SwerveSample const
  // &sample);

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return m_sysIdRoutineToApply->Quasistatic(direction);
  }

  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return m_sysIdRoutineToApply->Dynamic(direction);
  }

  void Periodic() override;

 private:
  void StartSimThread();
};

}  // namespace subsystems
