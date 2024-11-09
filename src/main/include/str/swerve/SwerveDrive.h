// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>

#include "constants/SwerveConstants.h"
#include "str/swerve/SwerveModule.h"

namespace str::swerve {
class SwerveDrive {
 public:
 private:
  std::array<SwerveModule, 4> modules{
      SwerveModule{consts::swerve::flModule,
                   consts::swerve::physicalCharacteristics,
                   consts::swerve::steerGains, consts::swerve::driveGains},
      SwerveModule{consts::swerve::frModule,
                   consts::swerve::physicalCharacteristics,
                   consts::swerve::steerGains, consts::swerve::driveGains},
      SwerveModule{consts::swerve::blModule,
                   consts::swerve::physicalCharacteristics,
                   consts::swerve::steerGains, consts::swerve::driveGains},
      SwerveModule{consts::swerve::brModule,
                   consts::swerve::physicalCharacteristics,
                   consts::swerve::steerGains, consts::swerve::driveGains}};

  ctre::phoenix6::hardware::Pigeon2 imu{consts::swerve::can_ids::IMU, "*"};
  ctre::phoenix6::sim::Pigeon2SimState& imuSimState = imu.GetSimState();

  std::array<ctre::phoenix6::BaseStatusSignal*, 34> allSignals;
};
}  // namespace str::swerve
