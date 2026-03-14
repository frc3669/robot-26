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

#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include <ctre/phoenix6/configs/MotionMagicConfigs.hpp>

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

    // Hood Angle 
    double m_hoodAngle = 10.0;
    double m_hoodAngleDelta = 0.0;

    // RPS Setting for Shooter Motors (TBD)  (Forward +value, Reverse -value)
    double m_shooterRPS = 2.0;
    double m_shooterRPSDelta = 0.0;

    // RPS Setting for Feeder Motors (TBD)
    double m_feederRPS = 3.0;

    // RPS Setting for Spindexer Motor (TBD)
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
  
    double m_turretTurns = 0;
    double m_turretAngle = 0.0;   // Testing
    double m_MinTurretAngle = -120.0; // MIN CCW
    double m_MaxTurretAngle = 120.0;  // MAX CW

    // Hood Motor (Position - ZERO to +VAL)
    ctre::phoenix6::hardware::TalonFX hoodMotor{42, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configHoodMotor{};
   
    double m_hoodTurns = 0;
   
    double m_MinHoodAngle = 10.0; // MIN (Ten Degrees) is Fully Retracted
    double m_MaxHoodAngle = 45.0; // MAX TBD is fully extended

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

    // Various Gear Ratios for the various motors
    // POSITION
    double m_turretGearRatio = (200 / 20) * (5 / 1); // Turret Base has 200 teeth, 
                                                     // Turret Motor has 20 teeth 
                                                     // Gearbox on motor (5-to-1)
    double m_hoodGearRatio = (350 / 16) * (30 / 12); // Hood "circle" has 350 teeth                          (54.6875 to 1)
                                                     // Hood triple-sprocket has 16 teeth
                                                     // Hood large belt socket has 30 teeth
                                                     // Hood small belt socket has 12 teeth
    // SPEED 
    double m_shooterGearRatio   = 1.0;
    double m_feederGearRatio    = 1.0;
    double m_spindexerGearRatio = ( 3 / 1 );         // Gearbox on motor (3-to-1)
    double m_intakeGearRatio    = 1.0;

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
    struct ShotSolutionEntry {
      double shooter_RPS;
      double hood_ANGLE;
    };
    // ***** 
    // Robot Shooting Solutions (SHOOTER RPS, and HOOD ANGLE) using Distance (in meters)
    //  
    // MAP INDEX - Range from 0 to INT (distance * 3), so each entry is 1/3 meter
    // *****
    const int MAX_SHOTMAP_INDEX = 50;
    map<int32_t, ShotSolutionEntry> m_shotSolutionMap = {
        { 0, { 0.0,     0.0 }},       // Distance = ZERO, Use SmartDashboard Variables m_ShooterRPS, m_hoodAngle.
        { 1, { 0.0,    15.0 }},
        { 2, { 5.0,    15.0 }},
        { 3, { 5.0,    15.0 }},
        { 4, { 5.0,    15.0 }},
        { 5, { 5.0,    15.0 }},
        { 6, { 5.0,    15.0 }},
        { 7, { 5.0,    15.0 }},
        { 8, { 5.0,    15.0 }},
        { 9, { 5.0,    15.0 }},
        {10, { 5.0,    15.0 }},
        {11, { 5.0,    15.0 }},
        {12, { 5.0,    15.0 }},
        {13, { 5.0,    15.0 }},
        {14, { 5.0,    15.0 }},
        {15, { 5.0,    15.0 }},
        {16, { 5.0,    15.0 }},
        {17, { 5.0,    15.0 }},
        {18, { 5.0,    15.0 }},
        {19, { 5.0,    15.0 }},
        {20, { 5.0,    15.0 }},
        {21, { 5.0,    15.0 }},
        {22, { 5.0,    15.0 }},
        {23, { 5.0,    15.0 }},
        {24, { 5.0,    15.0 }},
        {25, { 5.0,    15.0 }},
        {26, { 5.0,    15.0 }},
        {27, { 5.0,    15.0 }},
        {28, { 5.0,    15.0 }},
        {29, { 5.0,    15.0 }},
        {30, { 5.0,    15.0 }},
        {31, { 5.0,    15.0 }},
        {32, { 5.0,    15.0 }},
        {33, { 5.0,    15.0 }},
        {34, { 5.0,    15.0 }},
        {35, { 5.0,    15.0 }},
        {36, { 5.0,    15.0 }},
        {37, { 5.0,    15.0 }},
        {38, { 5.0,    15.0 }},
        {39, { 5.0,    15.0 }},
        {40, { 5.0,    15.0 }},
        {41, { 5.0,    15.0 }},
        {42, { 5.0,    15.0 }},
        {43, { 5.0,    15.0 }},
        {44, { 5.0,    15.0 }},
        {45, { 5.0,    15.0 }},
        {46, { 5.0,    15.0 }},
        {46, { 5.0,    15.0 }},
        {47, { 5.0,    15.0 }},
        {48, { 5.0,    15.0 }},
        {49, { 5.0,    15.0 }},
        {50, { 10.0,   50.5 }}
    };


    void setTurretPosition (double angle);
    void zeroizeTurretPosition ();
    void startTurret ();
    void stopTurret ();

    double getHoodAngle (double distance);
    double getHoodAngleDelta(frc::Pose2d robotPose);
    void setHoodPosition (double angle);   // Valid Hood angles are 10 degrees (retacted)  - N degrees (fully extended).  
    void zeroizeHoodPosition ();           // Zeroized is fully retracted which is 10 degree angle
    void startHood ();
    void stopHood ();

    double getShooterRPS(double distance);
    double getShooterRPSDelta(frc::Pose2d robotPose);
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
    void enableTopEndOperation ();

    void disableTurretOperation ();
    void disableHoodOperation ();
    void disableShooterOperation ();
    void disableFeederOperation ();
    void disableSpindexerOperation (); 
    void disableIntakeOperation ();
    void disableTopEndOperation ();
    
};