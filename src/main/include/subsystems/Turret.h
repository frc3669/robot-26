#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/SwerveModule.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/Field2d.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include "Constants.h"
#include "util.h"

#include "ctre/phoenix6/controls/VelocityDutyCycle.hpp"
#include "units/angular_velocity.h"

#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/controls/PositionDutyCycle.hpp"

#include "Swerve.h"

class Turret : public frc2::SubsystemBase {
  public:
    Turret(Swerve * drivePtr, frc2::CommandGenericHID *xkeys);

    void Periodic() override;
    void SimulationPeriodic() override;

    ~Turret();

    // Pointer for Access to the Swerve Subsystem
    Swerve * m_drivePtr;

    // ****************************************
    // Target Shooter Chooser
    frc::SendableChooser<string> m_shooterTgtChooser;
    string m_lastTgtSelection = "BLUEHub";
    //
    // Command Action Chooser
    frc::SendableChooser<string> m_cmdActionChooser;
    string m_lastCmdAction = "NoAction";


    frc2::CommandPtr cmdOnTurret();
    frc2::CommandPtr cmdOnHood();
    frc2::CommandPtr cmdOnShooter();
    frc2::CommandPtr cmdOnFeeder();
    frc2::CommandPtr cmdOnSpindexer();
    frc2::CommandPtr cmdOnIntake();
    frc2::CommandPtr cmdOnDeploy();

    frc2::CommandPtr cmdOffTurret();
    frc2::CommandPtr cmdOffHood();
    frc2::CommandPtr cmdOffShooter();
    frc2::CommandPtr cmdOffFeeder();
    frc2::CommandPtr cmdOffSpindexer();
    frc2::CommandPtr cmdOffIntake();
    frc2::CommandPtr cmdOffDeploy();

    // Indicates whether the Turret Target Pose has been SET
    bool m_turretTargetSet;

    // Turret Angle to the currently selected target 
    double m_turretTargetAngle;
    double m_turretTargetAngleDelta;
    // Turret Distance to the currently selected target
    double m_turretTargetDistance;
    double m_turretTargetDistanceDelta;

    // RPM Setting for Shooter Motors (TBD)  (Forward +value, Reverse -value)
    double m_shooterRPS;

    // RPM Setting for Feeder Motors (TBD)
    double m_feederRPS = 3.0;

    // RPM Setting for Spindexer Motor (TBD)
    double m_spindexerRPS = 3.4;

    // RPM setting for Intake Motor (TBD)
    double m_intakeRPM = 10;

    // Deploy Position for Intake Deploy Motor (TBD)
    double m_intakeDeployPosition = 10;
  

    // 
    // The currently selected turret target position
    frc::Translation2d m_turretTarget {0_in, 0_in};

     // Pre-defined TurretTargetPose Selections  (X, Y) in INCHES
    frc::Translation2d m_BLUE_TargetHub{182.11_in, 158.845_in};
    frc::Translation2d m_RED_TargetHub {469.11_in, 158.845_in};   

    // Set/Get Turret Target Location (X,Y)
    void setTurretTarget (frc::Translation2d theShooterTarget); 
    frc::Translation2d getTurretTarget ();   
    // ****************************************



  private:
    frc2::CommandGenericHID *xkeys;

    // **********************
    // Turret Related Motors
    // Turret Motor (Position - -VAL to ZERO to +VAL)
    ctre::phoenix6::hardware::TalonFX turretMotor{41, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configTurretMotor{};
    double m_turretRPS = 0;

    // Hood Motor (Position - ZERO to +VAL)
    ctre::phoenix6::hardware::TalonFX hoodMotor{42, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configHoodMotor{};
    double m_MaxHoodAngle = 1.0; // TBD 

    // Shooter Motors (Variable Speed) IDENTICAL SPEED, OPPOSITE DIRECTION
    // REVERSE
    ctre::phoenix6::hardware::TalonFX shooterReverseMotor{43, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configShooterReverseMotor{};
    // FORWARD
    ctre::phoenix6::hardware::TalonFX shooterForwardMotor{44, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configShooterForwardMotor{};


    // Feeder Motors (FIXED Speed) - IDENTICAL SPEED, OPPOSITE DIRECTION
    // DUAL WHEEL
    ctre::phoenix6::hardware::TalonFX feederDualMotor{45, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configFeederDualMotor{};
    // SINGLE WHEEL
    ctre::phoenix6::hardware::TalonFX feederSingleMotor{46, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configFeederSingleMotor{};

    // Spindexer Motor (FIXED Speed)
    ctre::phoenix6::hardware::TalonFX spindexerMotor{47, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configSpindexerMotor{};

    // Intake Device Motors (Intake - FIXED Speed and Deploy - Position)
    // Intake Motor (FIXED Speed)
    ctre::phoenix6::hardware::TalonFX intakeMotor{48, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configIntakeMotor{};
    double m_intakeRPS = 0;

    // Deploy Motor (Position - ZERO to +VAL)
    ctre::phoenix6::hardware::TalonFX deployMotor{49, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configDeployMotor{};


    bool isTurretActive;    // MUST be TRUE to shoot balls (when FALSE, the turret motor is STOPPED)
    bool isHoodActive;      // MUST be TRUE to shoot balls (when FALSE, the hood motor is LOWERED)
    bool isShooterActive;   // MUST be TRUE to shoot balls (when FALSE, the (2) shooter motor are STOPPED)
    bool isFeederActive;    // MUST be TRUE to feed balls  (when FALSE, the (2) feeder motors are STOPPED)
    bool isSpindexerActive; // MUST be TRUE to feed balls  (when FALSE, the spindexer is STOPPED)
    bool isIntakeActive;    // MUST be TRUE to pickup balls (when FALSE, the intake is UP and STOPPED)

    double m_turretGearRatio = 50;
    double m_hoodGearRatio = 2.0;
    double m_shooterGearRatio = 1.0;
    double m_feederGearRatio = 1.0;
    
    double m_intakeGearRatio = 1.0;

    // target pose for autonomous positioning during teleop
    frc::Pose2d m_targetPose;
    double lastTurretAngle = 0;

    // ****************************************
    // Turret Pose (Determined from Robot Pose and turret placement offsets)
    // NOTE: Robot Pose is Front Facing, with position at the Center of the Robot
    frc::Pose2d m_turretPose;

    // Relative Turret Offset (x,y in meters) from Robot Center, Pointing the SAME DIRECTION as the Robot.
    // (Since facing the same direction as the robot, angle is ZERO).
    // (Translates inches to meters accomplished by the type definitions)
    // (TBD - X= -12.0 inches, Y = +12.0 inches, NO ROTATION !!!)
    frc::Translation2d m_turretTranslation{ units::length::meter_t {-6.75_in},
                                            units::length::meter_t {+6.75_in}};

                    
    // ****************************************

    void setTurretPosition (double angle);
    void zeroizeTurretPosition ();
    void startTurret ();
    void stopTurret ();

    void setHoodPosition (double distance);
    void zeroizeHoodPosition ();
    void startHood ();
    void stopHood ();

    double getShooterRPS(double distance);
    void setShooterRPS (double rps);
    void startShooter ();
    void stopShooter ();

    void setFeederRPS ();
    void startFeeder ();
    void stopFeeder ();
   
    void setSpindexerRPS ();
    void startSpindexer ();
    void stopSpindexer ();

    void setIntakeRPM ();
    void zeroizeIntakePosition ();
    void startIntake ();
    void stopIntake ();

    
    double computeDistanceInMeters(double x1, double y1, double x2, double y2);

    double computeTurretAngleInDegrees(frc::Pose2d robotPose, frc::Translation2d turretTargetPose );

    // Enable / Disable Processing from User Interface Buttons
    void enableTurretOperation ();
    void enableHoodOperation ();
    void enableShooterOperation ();
    void enableFeederOperation ();
    void enableSpindexerOperation ();
    void enableIntakeOperation ();

    void disableTurretOperation ();
    void disableHoodOperation ();
    void disableShooterOperation ();
    void disableFeederOperation ();
    void disableSpindexerOperation (); 
    void disableIntakeOperation ();
    
};