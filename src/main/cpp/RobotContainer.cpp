// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

#include "constants/SwerveConstants.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "str/DriverstationUtils.h"
#include "subsystems/Drive.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  tuningTable->PutBoolean("SteerPidTuning", false);
  tuningTable->PutBoolean("DrivePidTuning", false);
  tuningTable->PutBoolean("SteerSysId", false);
  tuningTable->PutBoolean("DriveSysId", false);
  tuningTable->PutBoolean("Quasistatic", true);
  tuningTable->PutBoolean("Forward", true);

  steerTuneBtn.OnTrue(
      driveSub.TuneSteerPID([this] { return !steerTuneBtn.Get(); }));
  driveTuneBtn.OnTrue(
      driveSub.TuneDrivePID([this] { return !driveTuneBtn.Get(); }));

  steerSysIdBtn.WhileTrue(SteerSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));
  driveSysIdBtn.WhileTrue(DriveSysIdCommands(
      [this] { return tuningTable->GetBoolean("Forward", true); },
      [this] { return tuningTable->GetBoolean("Quasistatic", true); }));

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

frc2::CommandPtr RobotContainer::SteerSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(
          driveSub.SysIdSteerQuasistaticVoltage(
              frc2::sysid::Direction::kForward),
          driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kForward),
          quasistatic),
      frc2::cmd::Either(
          driveSub.SysIdSteerQuasistaticVoltage(
              frc2::sysid::Direction::kReverse),
          driveSub.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kReverse),
          quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::DriveSysIdCommands(
    std::function<bool()> fwd, std::function<bool()> quasistatic) {
  return frc2::cmd::Either(
      frc2::cmd::Either(
          driveSub.SysIdDriveQuasistaticVoltage(
              frc2::sysid::Direction::kForward),
          driveSub.SysIdDriveDynamicVoltage(frc2::sysid::Direction::kForward),
          quasistatic),
      frc2::cmd::Either(
          driveSub.SysIdDriveQuasistaticVoltage(
              frc2::sysid::Direction::kReverse),
          driveSub.SysIdDriveDynamicVoltage(frc2::sysid::Direction::kReverse),
          quasistatic),
      fwd);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

Drive& RobotContainer::GetDrive() {
  return driveSub;
}
