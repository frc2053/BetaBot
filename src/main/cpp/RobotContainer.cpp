// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  drivetrain.SetDefaultCommand(drivetrain.ApplyRequest([this] {
    return drive.WithVelocityX(-driverJoystick.GetLeftY() * MaxSpeed)
        .WithVelocityY(-driverJoystick.GetLeftX() * MaxSpeed)
        .WithRotationalRate(-driverJoystick.GetRightX() * MaxAngularRate);
  }));

  (driverJoystick.Back() && driverJoystick.Y())
      .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  (driverJoystick.Back() && driverJoystick.X())
      .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
  (driverJoystick.Start() && driverJoystick.Y())
      .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (driverJoystick.Start() && driverJoystick.X())
      .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  drivetrain.RegisterTelemetry(
      [this](auto const& state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
