// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/Threads.h>
#include <frc2/command/CommandScheduler.h>

#include "constants/SwerveConstants.h"

Robot::Robot() {
  // DANGEROUS MAKE SURE CODE DOESN'T BLOCK!!!
  frc::SetCurrentThreadPriority(true, 15);
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  AddPeriodic([this] { swerve.UpdateOdom(); },
              1 / consts::swerve::ODOM_UPDATE_RATE, 2_ms);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  swerve.UpdateSimulation();
  swerve.UpdateNTEntries();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
