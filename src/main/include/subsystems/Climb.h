// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

class Climb : public frc2::SubsystemBase {
public:
    Climb();
    frc2::CommandPtr extend();
    frc2::CommandPtr retract();
    frc2::CommandPtr brake();
    void Periodic() override;
    void SimulationPeriodic() override;

private:
    ctre::phoenix6::hardware::TalonFX mClimb{43, ctre::phoenix6::CANBus("rio")};
};