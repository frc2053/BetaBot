// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/Drive.h"
#include <frc2/command/button/NetworkButton.h>

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  Drive& GetDrive();

 private:
  void ConfigureBindings();

  frc2::CommandXboxController driverJoystick{0};

  Drive driveSub;

  std::shared_ptr<nt::NetworkTable> tuningTable{
      nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};
  frc2::NetworkButton steerTuneBtn{tuningTable, "SteerPidTuning"};
  frc2::NetworkButton driveTuneBtn{tuningTable, "DrivePidTuning"};
};
