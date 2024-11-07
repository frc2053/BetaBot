// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <str/GainTypes.h>
#include <str/Units.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/voltage.h>

#include <string>
#include <string_view>

#include "units/angular_velocity.h"

namespace str::swerve {
struct ModuleConstants {
  const std::string moduleName;
  const int driveId;
  const int steerId;
  const int encoderId;
  const units::turn_t steerEncoderOffset;
  const bool invertDrive;
  const bool invertSteer;

  ModuleConstants() = delete;
  ModuleConstants(std::string_view name, int dId, int sId, int eId,
                  units::turn_t offset, bool invDrive, bool invSteer)
      : moduleName(name),
        driveId(dId),
        steerId(sId),
        encoderId(eId),
        steerEncoderOffset(offset),
        invertDrive(invDrive),
        invertSteer(invSteer) {}
};

struct ModulePhysicalCharacteristics {
  const units::scalar_t steerGearing;
  const units::scalar_t driveGearing;
  const units::ampere_t steerSupplySideLimit;
  const units::ampere_t driveSupplySideLimit;
  const units::ampere_t steerStatorCurrentLimit;
  const units::ampere_t driveStatorCurrentLimit;
  const frc::DCMotor steerMotor;
  const frc::DCMotor driveMotor;
  const units::scalar_t couplingRatio;
  const units::meter_t wheelRadius;
  // Used for sim only
  const units::volt_t driveFrictionVoltage{0.25_V};
  const units::volt_t steerFrictionVoltage{0.25_V};

  ModulePhysicalCharacteristics() = delete;
  ModulePhysicalCharacteristics(
      units::scalar_t steerGear, units::scalar_t driveGear,
      units::ampere_t steerSupplyLim, units::ampere_t driveSupplyLim,
      units::ampere_t steerStatorLim, units::ampere_t driveStatorLim,
      const frc::DCMotor& steer, const frc::DCMotor drive,
      units::scalar_t coupling, units::meter_t wheelRad)
      : steerGearing{steerGear},
        driveGearing{driveGear},
        steerSupplySideLimit{steerSupplyLim},
        driveSupplySideLimit{driveSupplyLim},
        steerStatorCurrentLimit{steerStatorLim},
        driveStatorCurrentLimit{driveStatorLim},
        steerMotor{steer},
        driveMotor{drive},
        couplingRatio{coupling},
        wheelRadius{wheelRad} {}
};

struct SteerGains {
  const units::turns_per_second_t motionMagicCruiseVel;
  const str::gains::radial::turn_volt_ka_unit_t motionMagicExpoKa;
  const str::gains::radial::turn_volt_kv_unit_t motionMagicExpoKv;
  const str::gains::radial::turn_amp_ka_unit_t kA;
  const str::gains::radial::turn_amp_kv_unit_t kV;
  const units::ampere_t kS;
  const str::gains::radial::turn_amp_kp_unit_t kP;
  const str::gains::radial::turn_amp_ki_unit_t kI;
  const str::gains::radial::turn_amp_kd_unit_t kD;

  SteerGains() = delete;
  SteerGains(units::turns_per_second_t mmCv,
             str::gains::radial::turn_volt_ka_unit_t mmKa,
             str::gains::radial::turn_volt_kv_unit_t mmKv,
             str::gains::radial::turn_amp_ka_unit_t ka,
             str::gains::radial::turn_amp_kv_unit_t kv, units::ampere_t ks,
             str::gains::radial::turn_amp_kp_unit_t kp,
             str::gains::radial::turn_amp_ki_unit_t ki,
             str::gains::radial::turn_amp_kd_unit_t kd)
      : motionMagicCruiseVel{mmCv},
        motionMagicExpoKa{mmKa},
        motionMagicExpoKv{mmKv},
        kA{ka},
        kV{kv},
        kS{ks},
        kP{kp},
        kI{ki},
        kD{kd} {}

  bool operator==(const SteerGains& rhs) const {
    return units::essentiallyEqual(motionMagicCruiseVel,
                                   rhs.motionMagicCruiseVel, 1e-6),
           units::essentiallyEqual(motionMagicExpoKa, rhs.motionMagicExpoKa,
                                   1e-6),
           units::essentiallyEqual(motionMagicExpoKv, rhs.motionMagicExpoKv,
                                   1e-6),
           units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
               units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
               units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
               units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
               units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
               units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const SteerGains& rhs) const { return !operator==(rhs); }
};

struct DriveGains {
  const str::gains::radial::turn_amp_ka_unit_t kA;
  const str::gains::radial::turn_amp_kv_unit_t kV;
  const units::ampere_t kS;
  const str::gains::radial::turn_amp_kp_unit_t kP;
  const str::gains::radial::turn_amp_ki_unit_t kI;
  const str::gains::radial::turn_amp_kd_unit_t kD;

  DriveGains() = delete;
  DriveGains(str::gains::radial::turn_amp_ka_unit_t ka,
             str::gains::radial::turn_amp_kv_unit_t kv, units::ampere_t ks,
             str::gains::radial::turn_amp_kp_unit_t kp,
             str::gains::radial::turn_amp_ki_unit_t ki,
             str::gains::radial::turn_amp_kd_unit_t kd)
      : kA{ka}, kV{kv}, kS{ks}, kP{kp}, kI{ki}, kD{kd} {}

  bool operator==(const DriveGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const DriveGains& rhs) const { return !operator==(rhs); }
};
}  // namespace str::swerve
