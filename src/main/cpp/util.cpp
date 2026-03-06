#include <iostream>
#include <util.h>
#include <math.h>
#include "Constants.h"

using namespace ctre::phoenix6;
using namespace std;

void Util::configureMotor(hardware::TalonFX & motor, configs::TalonFXConfiguration const & config) {
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = motor.GetConfigurator().Apply(config);
        if (status.IsOK()) break;
    }
    if (!status.IsOK()) {
        std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    }
}

void Util::configureMotor(hardware::TalonFXS & motor, configs::TalonFXSConfiguration const & config) {
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = motor.GetConfigurator().Apply(config);
        if (status.IsOK()) break;
    }
    if (!status.IsOK()) {
        std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    }
}

/* 
Rather than reducing all the module speeds, this function just reduces
the given chassis speeds such that the module speeds, when recalculated, will be
desaturated. This is intended to be used on the target robot oriented chassisSpeeds before a
slew limiter is applied. The return value can then be reused to desaturate the field oriented
chassisSpeeds which is to be used with the slew limiter 
*/
double Util::desaturateChassisSpeeds(frc::ChassisSpeeds & robotSpeeds, wpi::array<frc::SwerveModuleState, 4U> const & states) {
    double fastestModuleSpeed = SwerveConstants::max_m_per_sec.value();
    for (auto & state : states) {
        if (state.speed.value() > fastestModuleSpeed) {
            fastestModuleSpeed = state.speed.value();
        }
    }
    double desaturationMultiplier = SwerveConstants::max_m_per_sec.value() / fastestModuleSpeed;
    robotSpeeds = robotSpeeds * desaturationMultiplier;
    return desaturationMultiplier;
}

Util::SlewLimiter::SlewLimiter() {}

void Util::SlewLimiter::Reset() {
    slewSpeeds = frc::ChassisSpeeds{};
}

frc::ChassisSpeeds Util::SlewLimiter::GetSpeeds() {
    return slewSpeeds;
}

void Util::SlewLimiter::Run(const frc::ChassisSpeeds &targetSpeeds, const units::time::second_t &secondsToFullSpeed, const units::time::second_t &period) {
    double distance2d = hypot(targetSpeeds.vx.value()-slewSpeeds.vx.value(), targetSpeeds.vy.value()-slewSpeeds.vy.value());
    double angularRateDifference = abs(targetSpeeds.omega.value()-slewSpeeds.omega.value());
    double incrementSizeTranslational = SwerveConstants::max_m_per_sec.value() * period / secondsToFullSpeed;
    double incrementSizeAngular = SwerveConstants::max_rad_per_sec.value() * period / secondsToFullSpeed;
    if (distance2d < incrementSizeTranslational) {
        slewSpeeds.vx = targetSpeeds.vx;
        slewSpeeds.vy = targetSpeeds.vy;
    } else {
        slewSpeeds.vx += (targetSpeeds.vx-slewSpeeds.vx) / distance2d * incrementSizeTranslational;
        slewSpeeds.vy += (targetSpeeds.vy-slewSpeeds.vy) / distance2d * incrementSizeTranslational;
    }
    if (angularRateDifference < incrementSizeAngular) {
        slewSpeeds.omega = targetSpeeds.omega;
    } else {
        slewSpeeds.omega += (targetSpeeds.omega-slewSpeeds.omega) / angularRateDifference * incrementSizeAngular;
    }
}