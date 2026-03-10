#include "subsystems/SwerveModule.h"
#include <ctre/phoenix6/CANBus.hpp>
#include "Constants.h"
#include "angleMath.h"
#include "util.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <string>

using namespace ctre::phoenix6;
using namespace std;

// create a Swerve module object with specified position and ID
SwerveModule::SwerveModule(int moduleID) : 
        m_moduleID(moduleID), m_driveMotor(10 + moduleID, CANBus("Swerve CAN")),
        m_steeringMotor(20 + moduleID, CANBus("Swerve CAN")),
        m_encoder(30 + moduleID, CANBus("Swerve CAN")) {
    m_driveMotorTurns = new StatusSignal(m_driveMotor.GetPosition());
    m_driveMotorVelocity = new StatusSignal(m_driveMotor.GetVelocity());
    m_encoderTurns = new StatusSignal(m_encoder.GetAbsolutePosition());
    configs::TalonFXConfiguration cfg{};
    cfg.Slot0.kP = 5;
    cfg.Slot0.kS = 3;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstants::max_torque_current;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstants::max_torque_current;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = 20_A;
    cfg.CurrentLimits.SupplyCurrentLowerTime = 0_s;
    cfg.CurrentLimits.SupplyCurrentLimit = 20_A;
    cfg.CurrentLimits.StatorCurrentLimit = 80_A;
    cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    Util::configureMotor(m_driveMotor, cfg);
    // apply configs for the steering motor
    configs::TalonFXConfiguration steeringCfg{};
    steeringCfg.ClosedLoopGeneral.ContinuousWrap = true;
    steeringCfg.Feedback.SensorToMechanismRatio = SwerveConstants::motor_turns_per_steering_turn;
    steeringCfg.Slot0.kP = 7;
    steeringCfg.Slot0.kD = 0.1;
    steeringCfg.Slot0.kS = 0.04;
    steeringCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    steeringCfg.CurrentLimits.SupplyCurrentLowerLimit = 20_A;
    steeringCfg.CurrentLimits.SupplyCurrentLowerTime = 0_s;
    steeringCfg.CurrentLimits.SupplyCurrentLimit = 10_A;
    steeringCfg.CurrentLimits.StatorCurrentLimit = 40_A;
    steeringCfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    Util::configureMotor(m_steeringMotor, steeringCfg);
}

void SwerveModule::setDesiredState(frc::SwerveModuleState & referenceState) {
    units::degree_t encoderAngle{m_encoderTurns->GetValue()};
    auto targetAngle = referenceState.angle.Degrees();
    auto angleDifference = targetAngle-encoderAngle;
    am::limit(angleDifference);
    // simultaneously optimize module direction and reduce module speed when pointed in the wrong direction
    auto moduleSpeed = referenceState.speed*units::math::cos(angleDifference);
    if (units::math::abs(angleDifference) > 90_deg) {
        targetAngle += 180_deg;
    }
    m_steeringMotor.SetControl(m_position.WithPosition(targetAngle));
    m_driveMotor.SetControl(m_velocity
        .WithVelocity(moduleSpeed.value()*SwerveConstants::motor_turns_per_m*1_tps));
}

// set the drive motor to brake mode
void SwerveModule::brake() {
    m_driveMotor.SetControl(controls::NeutralOut());
    m_steeringMotor.SetControl(controls::StaticBrake());
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return {units::meter_t{m_driveMotorTurns->GetValueAsDouble()/SwerveConstants::motor_turns_per_m},
            units::radian_t{m_encoderTurns->GetValueAsDouble()}};
}

frc::SwerveModuleState SwerveModule::GetState() {
    return {units::meters_per_second_t{m_driveMotorVelocity->GetValueAsDouble()/SwerveConstants::motor_turns_per_m},
            units::radian_t{m_encoderTurns->GetValueAsDouble()}};
}

void SwerveModule::InitializeOdometry() {
    lastWheelDistance = units::meter_t{m_driveMotorTurns->GetValueAsDouble() / SwerveConstants::motor_turns_per_m};
    m_steeringMotor.SetPosition(m_encoderTurns->GetValue());
}

frc::Translation2d SwerveModule::GetDeltaTranslation() {
    auto angle = m_encoderTurns->GetValue();
    auto wheelDistance = units::meter_t{m_driveMotorTurns->GetValueAsDouble() / SwerveConstants::motor_turns_per_m};
    auto deltaPosition = wheelDistance - lastWheelDistance;
    lastWheelDistance = wheelDistance;
    return {deltaPosition*units::math::cos(angle), deltaPosition*units::math::sin(angle)};
}

SwerveModule::~SwerveModule() {
    delete m_driveMotorTurns;
    delete m_driveMotorVelocity;
    delete m_encoderTurns;
}