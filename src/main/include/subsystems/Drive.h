#pragma once

#include <frc2/command/SubsystemBase.h>
#include <functional>
#include "str/swerve/SwerveDrive.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"

class Drive : public frc2::SubsystemBase {
 public:
  Drive();
  void Periodic() override;
  void SimulationPeriodic() override;
  void UpdateOdom();

  frc2::CommandPtr DriveTeleop(
      std::function<units::meters_per_second_t()> xVel,
      std::function<units::meters_per_second_t()> yVel,
      std::function<units::radians_per_second_t()> omega);

  frc2::CommandPtr DriveRobotRel(
      std::function<units::meters_per_second_t()> xVel,
      std::function<units::meters_per_second_t()> yVel,
      std::function<units::radians_per_second_t()> omega);

 private:
  str::swerve::SwerveDrive swerveDrive{};
};