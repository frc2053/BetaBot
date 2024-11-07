// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include "str/swerve/SwerveModuleHelpers.h"

namespace consts::swerve {
inline const str::swerve::SwerveModuleConstants flModule{
    "FL", 2, 3, 4, -0.272949_tr, false, true};
inline const str::swerve::SwerveModuleConstants frModule{
    "FR", 5, 6, 7, 0.356201_tr, true, true};
inline const str::swerve::SwerveModuleConstants blModule{
    "BL", 8, 9, 10, 0.195068_tr, false, true};
inline const str::swerve::SwerveModuleConstants brModule{
    "BR", 11, 12, 13, -0.492676_tr, true, true};
}  // namespace consts::swerve
