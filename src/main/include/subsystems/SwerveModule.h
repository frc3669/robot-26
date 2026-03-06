#pragma once
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <complex.h>
#include "angleMath.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/controller/PIDController.h"
#include "frc/geometry/Translation2d.h"

class SwerveModule {
  public:
    SwerveModule(int moduleID);
    void brake();
    void resetEncoders();
    frc::SwerveModulePosition GetPosition();
    frc::SwerveModuleState GetState();
    frc::Translation2d GetDeltaTranslation();
    void InitializeOdometry();
    void setDesiredState(frc::SwerveModuleState & referenceState);
    ~SwerveModule();
    
    ctre::phoenix6::StatusSignal<units::angle::turn_t> * m_driveMotorTurns;
    ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> * m_driveMotorVelocity;
    ctre::phoenix6::StatusSignal<units::angle::turn_t> * m_encoderTurns;

  private:
    int m_moduleID;
    ctre::phoenix6::hardware::TalonFX m_driveMotor;
    ctre::phoenix6::hardware::TalonFX m_steeringMotor;
    ctre::phoenix6::hardware::CANcoder m_encoder;
	  ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocity{0_tps};
	  ctre::phoenix6::controls::PositionDutyCycle m_position{0_tr};
    units::meter_t lastWheelDistance = 0_m;
};