// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "constants/SwerveConstants.h"
#include "subsystems/Drive.h"
#include <frc/MathUtil.h>
#include "str/DriverstationUtils.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  tuningTable->PutBoolean("SteerPidTuning", false);
  tuningTable->PutBoolean("DrivePidTuning", false);

  steerTuneBtn.OnTrue(
      driveSub.TuneSteerPID([this] { return !steerTuneBtn.Get(); }));
  driveTuneBtn.OnTrue(
      driveSub.TuneDrivePID([this] { return !driveTuneBtn.Get(); }));

  driveSub.SetDefaultCommand(driveSub.DriveTeleop(
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftY(), .1) *
            consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
      },
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverJoystick.GetLeftX(), .1) *
            consts::swerve::physical::PHY_CHAR.MaxLinearSpeed());
      },
      [this] {
        return frc::ApplyDeadband<double>(-driverJoystick.GetRightX(), .1) *
               consts::swerve::physical::MAX_ROT_SPEED;
      }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

Drive& RobotContainer::GetDrive() {
  return driveSub;
}