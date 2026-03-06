#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/Timer.h>

namespace Util {
    void configureMotor(ctre::phoenix6::hardware::TalonFX & motor, ctre::phoenix6::configs::TalonFXConfiguration const & config);
    void configureMotor(ctre::phoenix6::hardware::TalonFXS & motor, ctre::phoenix6::configs::TalonFXSConfiguration const & config);
    double desaturateChassisSpeeds(frc::ChassisSpeeds & robotSpeeds, wpi::array<frc::SwerveModuleState, 4U> const & states);
    class SlewLimiter {
    public:
        SlewLimiter();
        void Run(const frc::ChassisSpeeds &targetSpeeds, const units::time::second_t &secondsToFullSpeed, const units::time::second_t &period);
        void Reset();
        frc::ChassisSpeeds GetSpeeds();
    private:
        frc::Timer cycleTimer;
        frc::ChassisSpeeds slewSpeeds;
    };
}