// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include "frc/system/plant/DCMotor.h"
#include "str/swerve/SwerveModuleHelpers.h"

namespace consts::swerve {
inline const str::swerve::ModuleConstants flModule{"FL",         2,     3,   4,
                                                   -0.272949_tr, false, true};
inline const str::swerve::ModuleConstants frModule{"FR",        5,    6,   7,
                                                   0.356201_tr, true, true};
inline const str::swerve::ModuleConstants blModule{"BL",        8,     9,   10,
                                                   0.195068_tr, false, true};
inline const str::swerve::ModuleConstants brModule{"BR",         11,   12,  13,
                                                   -0.492676_tr, true, true};

inline const str::swerve::ModulePhysicalCharacteristics physicalCharacteristics{
    (50.0 / 14.0) * (60.0 / 10.0),
    (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0),
    40_A,
    60_A,
    40_A,
    80_A,
    frc::DCMotor::Falcon500FOC(1),
    frc::DCMotor::KrakenX60(1),
    (50.0 / 16.0),
    1.9154_in};

inline const str::swerve::SteerGains steerGains{
    physicalCharacteristics.steerMotor.freeSpeed /
        physicalCharacteristics.steerGearing,
    str::gains::radial::turn_volt_ka_unit_t{.1},
    str::gains::radial::turn_volt_kv_unit_t{
        .12 * physicalCharacteristics.steerGearing.value()},
    str::gains::radial::turn_amp_ka_unit_t{.82395},
    str::gains::radial::turn_amp_kv_unit_t{2.40},
    4.7145_A,
    str::gains::radial::turn_amp_kp_unit_t{500},
    str::gains::radial::turn_amp_ki_unit_t{0},
    str::gains::radial::turn_amp_kd_unit_t{39.663},
};

inline const str::swerve::DriveGains driveGains{
    str::gains::radial::turn_amp_ka_unit_t{0},
    str::gains::radial::turn_amp_kv_unit_t{0},
    9_A,
    str::gains::radial::turn_amp_kp_unit_t{9},
    str::gains::radial::turn_amp_ki_unit_t{0},
    str::gains::radial::turn_amp_kd_unit_t{0},
};
}  // namespace consts::swerve
