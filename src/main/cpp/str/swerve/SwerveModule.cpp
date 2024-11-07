// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/swerve/SwerveModule.h"

#include "frc/Alert.h"

using namespace str::swerve;

SwerveModule::SwerveModule(const ModuleConstants& consts)
    : configureSteerAlert(consts.moduleName + " Steer Motor Configuration",
                          frc::Alert::AlertType::kError) {
  ConfigureSteerMotor();
}

void SwerveModule::ConfigureSteerMotor() {
  configureSteerAlert.Set(true);
}
