// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/Alert.h>
#include <units/angle.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "str/swerve/SwerveModuleHelpers.h"

namespace str::swerve {

class SwerveModule {
 public:
  explicit SwerveModule(const ModuleConstants& consts);

 private:
  void ConfigureSteerMotor();

  frc::Alert configureSteerAlert;
};
}  // namespace str::swerve
