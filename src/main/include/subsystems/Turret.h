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
#include <ctre/phoenix/StatusCodes.h>

#include "Swerve.h"

class Turret : public frc2::SubsystemBase {
  public:
    Turret(Swerve * drivePtr, frc2::CommandGenericHID *xkeys);

    void Periodic() override;
    void SimulationPeriodic() override;

    ~Turret();

    // Pointer for Access to the Swerve Subsystem
    Swerve * m_drivePtr;


    // Starting Pose Chooser
    frc::SendableChooser<frc::Pose2d> m_startPoseChooser; 
    frc::Pose2d m_lastStartingPose;
    frc::Pose2d m_newRobotPose;
    
    // ****************************************
    // Target Shooter Chooser
    frc::SendableChooser<string> m_shooterTgtChooser;
    string m_lastTgtSelection = "unknown";
    //
    // Command Action Chooser
    frc::SendableChooser<string> m_cmdActionChooser;
    string m_lastCmdAction = "NoAction";


    frc2::CommandPtr cmdOnTurret();
    frc2::CommandPtr cmdOnHood();
    frc2::CommandPtr cmdOnShooter();
    frc2::CommandPtr cmdOnFeeder();
    frc2::CommandPtr cmdOnSpindexer();
    frc2::CommandPtr cmdDeployIntake();
    frc2::CommandPtr cmdRaiseIntake();
    frc2::CommandPtr cmdOnTopEnd();

    frc2::CommandPtr cmdOffTurret();
    frc2::CommandPtr cmdOffHood();
    frc2::CommandPtr cmdOffShooter();
    frc2::CommandPtr cmdOffFeeder();
    frc2::CommandPtr cmdOffSpindexer();
    frc2::CommandPtr cmdRetractIntake();
    frc2::CommandPtr cmdReverseIntake();
    frc2::CommandPtr cmdOffTopEnd();
    frc2::CommandPtr cmdRevTopEnd();
    frc2::CommandPtr cmdManualOperation();
    frc2::CommandPtr cmdIntakeON_OFF();
    frc2::CommandPtr cmdUseCameras();
    

    // Indicates whether the Turret Target Pose has been SET
    bool m_turretTargetSet;
    int  m_shotTableIndex = 1;

    // Turret Angle (uncompensated) to the currently selected target 
    double m_turretToTargetAngle;
    double m_lastTurretAngle;
    // Turret Distance (meters) to the currently selected target
    double m_turretToTargetDistance;

    // Hood Angle 
    double m_hoodAngle = 25.3;

    // RPS Setting for Shooter Motors (TBD)  (Forward +value, Reverse -value)
    double m_shooterRPS = 52.6;

    // RPS Setting for Feeder Motors (TBD)
    double m_feederRPS = 50.0;

    // RPS Setting for Spindexer Motor (TBD)
    double m_spindexerRPS = 20.0;

    // RPS setting for Intake Motors (TBD)
    double m_intakeUpperRPS = 35;
    double m_intakeLowerRPS = 30;

    // Deploy and Raise Positions for Intake Deploy Motor (TBD)
    double m_intakeDeployPosition = 18.0;
    double m_intakeRaisePosition  =  8.0;
  
    // 
    // The currently selected turret target position
    frc::Translation2d m_turretTarget {0_in, 0_in};

     // Pre-defined TurretTargetPose Selections  (X, Y) in INCHES
    frc::Translation2d m_BLUE_TgtHub { 4.67_m, 4.07_m};
    frc::Translation2d m_BLUE_Outpost{ 2.3_m,  2.0_m};
    frc::Translation2d m_BLUE_Depot  { 2.3_m,  6.41_m};

    frc::Translation2d m_RED_TgtHub  { 12.02_m,  4.07_m}; 
    frc::Translation2d m_RED_Outpost { 14.33_m,  6.41_m};
    frc::Translation2d m_RED_Depot   { 14.33_m,  2.05_m};
    // Target Selection Mode 
    // Manual OR Automatic 
    bool m_isManualTgtSelection = false;

    // Set/Get Turret Target Location (X,Y)
    void setTurretTarget (frc::Translation2d theShooterTarget); 
    frc::Translation2d getTurretTarget ();   
    // ****************************************


  private:
    frc2::CommandGenericHID *xkeys;

    // Logic to flip the starting Pose when RED Alliance
    frc::Pose2d flipPose(const frc::Pose2d& pose);
    frc::Pose2d m_startingPose;

    // Shot Table Enable Flag
    bool m_isShotTableEnabled = false;
    bool m_isCompensationEnabled  = false;
    
    // Variable to assist in limiting when motors get commanded
    bool   m_isTurretClassConfigComplete = false;
    double m_lastCmdTurretAngle = 0;   
    bool   m_lastCfgIsSmallAngle = true;  // Initial Turret Config is for Small Angle
    double m_lastCmdHoodAngle = 0;   
    double m_lastCmdShooterRPS = 0;
    double m_lastCmdFeederRPS = 0;
    double m_lastCmdSpindexerRPS = 0;

    // **********************
    // Turret Related Motors
    // Turret Motor (Position - -VAL to ZERO to +VAL)
    ctre::phoenix6::hardware::TalonFX turretMotor{41, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configTurretMotor{};

    double m_turretTurns = 0;
    double m_turretAngle = 0.0;   // Testing
    double MAX_TURRET_ROTATION_ANGLE = 180;    // Used for calculating the desired turret angle +/- 180
    double m_MinTurretAngle = -180.0;          // MIN CCW  - Actual Limit which we can command
    double m_MaxTurretAngle = 180.0;           // MAX CW   - Actual Limit which we can command

    // Hood Motor (Position - ZERO to +VAL)
    ctre::phoenix6::hardware::TalonFX hoodMotor{42, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configHoodMotor{};  

    double m_hoodTurns = 0;
    double m_MinHoodAngle = 10.0; // MIN (Ten Degrees) is Fully Retracted
    double m_MaxHoodAngle = 50.0; // MAX TBD is fully extended

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

    // Intake Device Motors (Intake - FIXED 2 Speed and 1 Deploy - Position)
    // Intake Upper Motor (FIXED Speed)
    ctre::phoenix6::hardware::TalonFX intakeUpperMotor{48, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configIntakeUpperMotor{};
     // Intake Lower Motor (FIXED Speed)
    ctre::phoenix6::hardware::TalonFX intakeLowerMotor{49, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configIntakeLowerMotor{};

    // Intake Deploy Motor (Position - ZERO to +VAL)
    ctre::phoenix6::hardware::TalonFX deployMotor{50, ctre::phoenix6::CANBus("Main CAN")};
    ctre::phoenix6::configs::TalonFXConfiguration configDeployMotor{};


    bool isTurretActive;    // MUST be TRUE to shoot balls (when FALSE, the turret motor is STOPPED)
    bool isHoodActive;      // MUST be TRUE to shoot balls (when FALSE, the hood motor is LOWERED)
    bool isShooterActive;   // MUST be TRUE to shoot balls (when FALSE, the (2) shooter motor are STOPPED)
    bool isFeederActive;    // MUST be TRUE to feed balls  (when FALSE, the (2) feeder motors are STOPPED)
    bool isFeederReversed;
    bool isSpindexerActive; // MUST be TRUE to feed balls  (when FALSE, the spindexer is STOPPED)
    bool isSpindexerReversed;
    bool isIntakeActive;    // MUST be TRUE to pickup balls (when FALSE, the intake is UP and STOPPED)
    bool isIntakeDeployed;
    // Reversal Button(s) - Reverses on push, drives forward on next push.
    bool isTopEndReversed;  // Reverses TopEnd Motors (Loader and Spindexer)  - Shooter is NOT reversed
    bool isIntakeReversed;  // Reverses Intake Motors (Upper and Lower Intake) - 
    bool isManualOperation; // Select Manual Setting of all Top End Robot Motors
    bool isCamerasInUse;    // Whether Limelight Cameras are in use

    bool isTopEndActive;           // Set TRUE when the Top End has been turned on
    bool isTurretDeadZoneDisabled; // Set TRUE when the Top End was enabled, and then entered the dead zone.
                                   // Turret Dead Zone is past the turret aiming angle.
                                   // If we entered a dead zone when the TopEnd was active, we need to re-enable
                                   // the TopEnd when we leave the dead zone.

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



    struct ShotSetpoint {
      double turret_AngleDegrees;
      double hood_AngleDegrees;
      double shooter_RPS;
    };

    // ****************************************
    struct ShotSolutionEntry {
      double shooter_RPS;
      double hood_ANGLE;
      double feeder_RPS;
      double spindexer_RPS;
    };
    // ***** 
    // Robot Shooting Solutions (SHOOTER RPS, and HOOD ANGLE) using Distance (in meters)
    //  
    // MAP INDEX - Range from 0 to INT (distance * 3), so each entry is 1/3 meter
    // Shots are expected from 3 meters to 8 meters ONLY.
    // *****
    const int MAX_SHOTMAP_INDEX = 50;
    map<int32_t, ShotSolutionEntry> m_shotSolutionMap = {
        { 3, { 48.0,    14.0,   47,   20  }},    // 1 meters
        { 4, { 48.7,    16.0,   47,   20  }},
        { 5, { 49.3,    18.0,   47,   20  }},
        { 6, { 50.0,    20.0,   47,   20  }},    // 2 meters
        { 7, { 51.3,    22.6,   47,   20  }},
        { 8, { 52.6,    25.3,   47,   20  }},
        { 9, { 54.0,    28.0,   47,   20  }},    // 3 meters
        {10, { 56.0,    29.0,   47,   20  }},
        {11, { 58.0,    30.0,   47,   20  }},
        {12, { 60.0,    31.0,   47,   20  }},    // 4 meters
        {13, { 63.0,    31.6,   47,   20  }},
        {14, { 66.0,    32.2,   47,   20  }},
        {15, { 69.0,    33.0,   47,   20  }},    // 5 meters
        {16, { 72.0,    33.7,   47,   20  }},
        {17, { 75.0,    34.3,   47,   20  }},
        {18, { 78.0,    35.0,   47,   20  }},    // 6 meters
        {19, { 81.3,    35.7,   47,   20  }},
        {20, { 84.6,    36.4,   47,   20  }},
        {21, { 88.0,    37.0,   50,   20  }},    // 7 meters
        {22, { 92.0,    39.0,   47,   20  }},
        {23, { 96.0,    41.0,   47,   20  }},
        {24, { 100.0,   43.0,   47,   20  }}    // 8 meters
    };


    void setTurretPosition (double angle);
    void zeroizeTurretPosition ();
    void startTurret ();
    void stopTurret ();

    double getHoodAngle (int shotTableIndex);
    double getHoodAngleDelta(frc::Pose2d robotPose);
    void setHoodPosition (double angle);   // Valid Hood angles are 10 degrees (retacted)  - N degrees (fully extended).  
    void zeroizeHoodPosition ();           // Zeroized is fully retracted which is 10 degree angle
    void startHood ();
    void stopHood ();

    double getShooterRPS(int shotTableIndex);
    void setShooterRPS (double rps);
    void startShooter ();
    void stopShooter ();

    double getFeederRPS(int shotTableIndex);
    void setFeederRPS (double rps);
    void startFeeder ();
    void stopFeeder ();
   
    double getSpindexerRPS(int shotTableIndex);
    void setSpindexerRPS (double rps);
    void startSpindexer ();
    void stopSpindexer ();

    void setIntakeRPS ();         // See m_intakeUpperRPS and m_intakeLowerRPS 
    void zeroizeIntakePosition ();
    void deployIntake ();
    void raiseIntake ();
    void retractIntake ();
    void intakeOnOff();

    
    double computeDistanceInMeters(double x1, double y1, double x2, double y2);

    double computeTurretToTgtAngleInDegrees(frc::Pose2d robotPose, frc::Translation2d turretTargetPose );
    ShotSetpoint getVelocityCompensatedShotSetpoint(double robotToTargetAngle, double shooterVelocityMPS, double hoodAngleDegrees);

    // Enable / Disable Processing from User Interface Buttons
    void enableTurretOperation ();
    void enableHoodOperation ();
    void enableShooterOperation ();
    void enableFeederOperation ();
    void enableSpindexerOperation ();
    void deployIntakeOperation ();
    void raiseIntakeOperation ();
    void enableTopEndOperation ();

    void disableTurretOperation ();
    void disableHoodOperation ();
    void disableShooterOperation ();
    void disableFeederOperation ();
    void disableSpindexerOperation (); 
    void retractIntakeOperation ();
    void disableTopEndOperation ();
    void reverseTopEndOperation();
    void reverseIntakeOperation();
    void manualOperation();
    void intakeOnOffOperation();
    void cameraOperation();
    
};