#include "subsystems/Drive.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

Drive::Drive() {}

void Drive::Periodic() {
  swerveDrive.UpdateNTEntries();
}

void Drive::SimulationPeriodic() {
  swerveDrive.UpdateSimulation();
}

void Drive::UpdateOdom() {
  swerveDrive.UpdateOdom();
}

frc2::CommandPtr Drive::DriveTeleop(
    std::function<units::meters_per_second_t()> xVel,
    std::function<units::meters_per_second_t()> yVel,
    std::function<units::radians_per_second_t()> omega) {
  return frc2::cmd::Run(
             [this, xVel, yVel, omega] {
               swerveDrive.DriveFieldRelative(xVel(), yVel(), omega(), true);
             },
             {this})
      .WithName("DriveTeleop");
}

frc2::CommandPtr Drive::DriveRobotRel(
    std::function<units::meters_per_second_t()> xVel,
    std::function<units::meters_per_second_t()> yVel,
    std::function<units::radians_per_second_t()> omega) {
  return frc2::cmd::Run(
             [this, xVel, yVel, omega] {
               swerveDrive.Drive(xVel(), yVel(), omega(), false);
             },
             {this})
      .WithName("DriveRobotRel");
}