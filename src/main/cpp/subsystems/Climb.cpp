// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climb.h"
#include "util.h"
#include <frc2/command/Commands.h>

using namespace ctre::phoenix6;

Climb::Climb() {
    configs::TalonFXConfiguration cfg{};
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    Util::configureMotor(mClimb, cfg);
}

frc2::CommandPtr Climb::extend() {
    return Run([this] { mClimb.Set(-1); }).WithName("Extending Climb");
}

frc2::CommandPtr Climb::retract() {
    return Run([this] { mClimb.Set(1); }).WithName("Retracting Climb");
}

frc2::CommandPtr Climb::brake() {
    return Run([this] { mClimb.SetControl(controls::NeutralOut()); }).WithName("Braking Climb");
}

void Climb::Periodic() {}

void Climb::SimulationPeriodic() {}