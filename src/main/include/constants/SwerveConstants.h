// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/frequency.h>

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/system/plant/DCMotor.h"
#include "str/swerve/SwerveModuleHelpers.h"
#include "units/angle.h"

namespace consts::swerve {

inline constexpr units::hertz_t ODOM_UPDATE_RATE = 250_Hz;

inline constexpr int IMU_ID = 14;
inline constexpr units::degree_t IMU_MOUNT_ROLL = 0_deg;
inline constexpr units::degree_t IMU_MOUNT_PITCH = 0_deg;
inline constexpr units::degree_t IMU_MOUNT_YAW = 0_deg;

inline const str::swerve::ModuleConstants FL_MODULE{"FL",         2,     3,   4,
                                                    -0.272949_tr, false, true};
inline const str::swerve::ModuleConstants FR_MODULE{"FR",        5,    6,   7,
                                                    0.356201_tr, true, true};
inline const str::swerve::ModuleConstants BL_MODULE{"BL",        8,     9,   10,
                                                    0.195068_tr, false, true};
inline const str::swerve::ModuleConstants BR_MODULE{"BR",         11,   12,  13,
                                                    -0.492676_tr, true, true};

inline const str::swerve::ModulePhysicalCharacteristics PHY_CHAR{
    (50.0 / 14.0) * (60.0 / 10.0),
    (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0),
    40_A,
    80_A,
    40_A,
    80_A,
    frc::DCMotor::Falcon500FOC(1),
    frc::DCMotor::KrakenX60FOC(1),
    (50.0 / 16.0),
    1.9154_in};

inline const str::swerve::SteerGains STEER_GAINS{
    PHY_CHAR.steerMotor.freeSpeed / PHY_CHAR.steerGearing,
    str::gains::radial::turn_volt_ka_unit_t{.1},
    str::gains::radial::turn_volt_kv_unit_t{.12 *
                                            PHY_CHAR.steerGearing.value()},
    str::gains::radial::turn_amp_ka_unit_t{.82395},
    str::gains::radial::turn_amp_kv_unit_t{2.40},
    4.7145_A,
    str::gains::radial::turn_amp_kp_unit_t{500},
    str::gains::radial::turn_amp_ki_unit_t{0},
    str::gains::radial::turn_amp_kd_unit_t{39.663},
};

inline const str::swerve::DriveGains DRIVE_GAINS{
    str::gains::radial::turn_amp_ka_unit_t{0},
    str::gains::radial::turn_amp_kv_unit_t{0},
    9_A,
    str::gains::radial::turn_amp_kp_unit_t{9},
    str::gains::radial::turn_amp_ki_unit_t{0},
    str::gains::radial::turn_amp_kd_unit_t{0},
};

inline constexpr units::meter_t WHEELBASE_WIDTH = 21.75_in;
inline constexpr units::meter_t WHEELBASE_LENGTH = 15.75_in;

inline constexpr std::array<frc::Translation2d, 4> MODULE_LOCATIONS{
    frc::Translation2d{WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2}};

inline frc::SwerveDriveKinematics<4> KINEMATICS{
    MODULE_LOCATIONS[0], MODULE_LOCATIONS[1], MODULE_LOCATIONS[2],
    MODULE_LOCATIONS[3]};

inline constexpr units::radians_per_second_t DRIVE_MAX_ROT_SPEED =
    540_deg_per_s;
}  // namespace consts::swerve
