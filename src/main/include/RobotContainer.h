// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "SwerveTelemetry.h"
#include "generated/TunerConstants.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  SwerveTelemetry logger{MaxSpeed};

  subsystems::CommandSwerveDrivetrain drivetrain{
      TunerConstants::CreateDrivetrain()};

 private:
  void ConfigureBindings();

  frc2::CommandXboxController driverJoystick{0};

  units::meters_per_second_t MaxSpeed =
      TunerConstants::kSpeedAt12Volts;  // kSpeedAt12Volts desired top speed
  units::radians_per_second_t MaxAngularRate =
      0.75_tps;  // 3/4 of a rotation per second max angular velocity

  swerve::requests::FieldCentric drive =
      swerve::requests::FieldCentric{}
          .WithDeadband(MaxSpeed * 0.1)
          .WithRotationalDeadband(MaxAngularRate * 0.1)  // Add a 10% deadband
          .WithDriveRequestType(
              swerve::SwerveModule::DriveRequestType::OpenLoopVoltage);
};
