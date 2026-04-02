#include "subsystems/Turret.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <math.h>
#include <thread>


Turret::Turret(Swerve * drivePtr, frc2::CommandGenericHID *xkeys) {   
                    
                    
    // add all the status signals to a list for syncronized updates
    m_drivePtr = drivePtr;   // Get access to the Swerve susbsystem, so the pose can be returned
    this->xkeys = xkeys;     // Get access to user interface keys

    // All Turret motors are STOPPED upon Robot start-up
    isTurretActive = false;     // MUST be TRUE to shoot balls (when FALSE, the turret is STOPPED)
    isHoodActive = false;       // MUST be TRUE to shoot balls (when FALSE, the hood is LOWERED)
    isShooterActive = false;    // MUST be TRUE to shoot balls (when FALSE, the shooter is STOPPED)
    isFeederActive = false;     // MUST be TRUE to feed balls  (when FALSE, the feeder is STOPPED)
    isSpindexerActive = false;  // MUST be TRUE to feed balls  (when FALSE, the spindexer is STOPPED)
    isIntakeActive = false;     // MUST be TRUE to intake balls (when FALSE, the intake is NOT Deployed and STOPPED)
    isTopEndActive = false;
    isTopEndReversed = false;   // Top End Reversed 
    isIntakeDeployed = false;
    isIntakeReversed = false;   // Intake is Reversed
    isTurretDeadZoneDisabled = false;
    isCamerasInUse = false;     // Whether Cameras ar in Use

    // ***************************************
    // define PID values for all Turret motors
    // Turret Motor (Position)
    // Slot 0 Gain settings
    configTurretMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake; 
    // Slot 0 settings
    configTurretMotor.Slot0.kS = 0.25;  // Add 0.25 V output to overcome static friction
    configTurretMotor.Slot0.kV = 0.12;  // A velocity target of 1 rps results in 0.12 V output
    configTurretMotor.Slot0.kA = 0.01;  // An acceleration of 1 rps/s requires 0.01 V output
    configTurretMotor.Slot0.kP = 4.8;   // A position error of 2.5 rotations results in 12 V output
    configTurretMotor.Slot0.kI = 0;     // no output for integrated error
    configTurretMotor.Slot0.kD = 0.1;   // A velocity error of 1 rps results in 0.1 V output   configTurretMotor.Slot0.kP = 1.2;
    // Motion Magic settings
    // SMALL ANGLE configuration settings
    configTurretMotor.MotionMagic.MotionMagicAcceleration = 0_tr_per_s_sq; // acceleration  (ramp)
    configTurretMotor.MotionMagic.MotionMagicCruiseVelocity = 6_tps;       // velocity  (once ramp up)
    configTurretMotor.MotionMagic.MotionMagicExpo_kV = (ctre::unit::volts_per_turn_per_second_t) 0.12; // Speed per unit of voltage (rotations/sec/V)
    configTurretMotor.MotionMagic.MotionMagicExpo_kA = (ctre::unit::volts_per_turn_per_second_squared_t)0.1; // Acceleration per unit of voltage (rotations/sec^2/V)  
    // Apply Configuration 
    turretMotor.GetConfigurator().Apply(configTurretMotor);
    m_lastCfgIsSmallAngle = true;

    // Hood Motor (Position)
    configHoodMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake; 
    configHoodMotor.Slot0.kP = 4.8;
    configHoodMotor.Slot0.kI = 0.155;
    configHoodMotor.Slot0.kD = 0.1;
    hoodMotor.GetConfigurator().Apply(configHoodMotor);


    // Shooter Motor(s) (Speed)  SAME CONFIG IS USED FOR BOTH MOTORS
    configShooterForwardMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configShooterForwardMotor.Slot0.kS = 0.1;
    configShooterForwardMotor.Slot0.kV = 0.12;
    configShooterForwardMotor.Slot0.kP = 0.11;
    configShooterForwardMotor.Slot0.kI = 0;
    configShooterForwardMotor.Slot0.kD = 0;
    shooterForwardMotor.GetConfigurator().Apply(configShooterForwardMotor);
    configShooterReverseMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configShooterReverseMotor.Slot0.kS = 0.1;
    configShooterReverseMotor.Slot0.kV = 0.12;
    configShooterReverseMotor.Slot0.kP = 0.11;
    configShooterReverseMotor.Slot0.kI = 0;
    configShooterReverseMotor.Slot0.kD = 0;
    shooterReverseMotor.GetConfigurator().Apply(configShooterReverseMotor);
 
    // Feeder Motor(s) (Speed)  SAME CONFIG IS USED FOR BOTH MOTORS
    configFeederDualMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configFeederDualMotor.Slot0.kP = 0.1;   // Proportional gain (adjust as needed)
    configFeederDualMotor.Slot0.kI = 0.0;   // Integrated error gain
    configFeederDualMotor.Slot0.kD = 0.01;  // Derivative gain
    configFeederDualMotor.Slot0.kV = 0.12;  // Feedforward gain (Volts per rotations per second, calculate with SysId)
    configFeederDualMotor.Slot0.kS = 0.25;  // Static feedforward gain (Volts to overcome static friction)
    auto& dualMotionMagic = configFeederDualMotor.MotionMagic;
    dualMotionMagic.MotionMagicAcceleration = 100_tr_per_s_sq; // Ramp up/down rate
    dualMotionMagic.MotionMagicJerk = 1000_tr_per_s_cu;        // Smoothing (optional)
   
    //configFeederDualMotor.Slot0.kS = 0.1;   
    //configFeederDualMotor.Slot0.kV = 0.12;
    //configFeederDualMotor.Slot0.kP = 0.11;
    //configFeederDualMotor.Slot0.kI = 0;
    //configFeederDualMotor.Slot0.kD = 0;
    feederDualMotor.GetConfigurator().Apply(configFeederDualMotor);
    configFeederSingleMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configFeederSingleMotor.Slot0.kP = 0.1;   // Proportional gain (adjust as needed)
    configFeederSingleMotor.Slot0.kI = 0.0;   // Integrated error gain
    configFeederSingleMotor.Slot0.kD = 0.01;  // Derivative gain
    configFeederSingleMotor.Slot0.kV = 0.12;  // Feedforward gain (Volts per rotations per second, calculate with SysId)
    configFeederSingleMotor.Slot0.kS = 0.25;  // Static feedforward gain (Volts to overcome static friction)
    auto& singleMotionMagic = configFeederSingleMotor.MotionMagic;
    singleMotionMagic.MotionMagicAcceleration = 100_tr_per_s_sq; // Ramp up/down rate
    singleMotionMagic.MotionMagicJerk = 1000_tr_per_s_cu;        // Smoothing (optional)
   
    //configFeederSingleMotor.Slot0.kS = 0.1; 
    //configFeederSingleMotor.Slot0.kV = 0.12;
    //configFeederSingleMotor.Slot0.kP = 0.11;
    //configFeederSingleMotor.Slot0.kI = 0;
    //configFeederSingleMotor.Slot0.kD = 0;
    feederSingleMotor.GetConfigurator().Apply(configFeederSingleMotor);
 
    // Spindexer Motor (Speed)
    configSpindexerMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configSpindexerMotor.Slot0.kP = 0.1;   // Proportional gain (adjust as needed)
    configSpindexerMotor.Slot0.kI = 0.0;   // Integrated error gain
    configSpindexerMotor.Slot0.kD = 0.01;  // Derivative gain
    configSpindexerMotor.Slot0.kV = 0.12;  // Feedforward gain (Volts per rotations per second, calculate with SysId)
    configSpindexerMotor.Slot0.kS = 0.25;  // Static feedforward gain (Volts to overcome static friction)
    auto& spindexerMotionMagic = configSpindexerMotor.MotionMagic;
    spindexerMotionMagic.MotionMagicAcceleration = 100_tr_per_s_sq; // Ramp up/down rate
    spindexerMotionMagic.MotionMagicJerk = 1000_tr_per_s_cu;        // Smoothing (optional)

//    configSpindexerMotor.Slot0.kS = 0.1;
//    configSpindexerMotor.Slot0.kV = 0.12;
//    configSpindexerMotor.Slot0.kP = 0.11;
//    configSpindexerMotor.Slot0.kI = 0;
//    configSpindexerMotor.Slot0.kD = 0;
    spindexerMotor.GetConfigurator().Apply(configSpindexerMotor);

    // Intake Motors (Upper, Lower, and Deploy)
    // Upper and Lower Motor(s) (Speed)  SAME CONFIG IS USED FOR BOTH MOTORS
    configIntakeUpperMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configIntakeUpperMotor.Slot0.kP = 0.1;   // Proportional gain (adjust as needed)
    configIntakeUpperMotor.Slot0.kI = 0.0;   // Integrated error gain
    configIntakeUpperMotor.Slot0.kD = 0.01;  // Derivative gain
    configIntakeUpperMotor.Slot0.kV = 0.12;  // Feedforward gain (Volts per rotations per second, calculate with SysId)
    configIntakeUpperMotor.Slot0.kS = 0.25;  // Static feedforward gain (Volts to overcome static friction)
    auto& intakeUpperMotionMagic = configIntakeUpperMotor.MotionMagic;
    intakeUpperMotionMagic.MotionMagicAcceleration = 100_tr_per_s_sq; // Ramp up/down rate
    intakeUpperMotionMagic.MotionMagicJerk = 1000_tr_per_s_cu;        // Smoothing (optional)

//    configIntakeUpperMotor.Slot0.kS = 0.1;   
//    configIntakeUpperMotor.Slot0.kV = 0.12;
//    configIntakeUpperMotor.Slot0.kP = 0.11;
//    configIntakeUpperMotor.Slot0.kI = 0;
//    configIntakeUpperMotor.Slot0.kD = 0;
    intakeUpperMotor.GetConfigurator().Apply(configIntakeUpperMotor);
    configIntakeLowerMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configIntakeLowerMotor.Slot0.kP = 0.1;   // Proportional gain (adjust as needed)
    configIntakeLowerMotor.Slot0.kI = 0.0;   // Integrated error gain
    configIntakeLowerMotor.Slot0.kD = 0.01;  // Derivative gain
    configIntakeLowerMotor.Slot0.kV = 0.12;  // Feedforward gain (Volts per rotations per second, calculate with SysId)
    configIntakeLowerMotor.Slot0.kS = 0.25;  // Static feedforward gain (Volts to overcome static friction)
    auto& intakeLowerMotionMagic = configIntakeLowerMotor.MotionMagic;
    intakeLowerMotionMagic.MotionMagicAcceleration = 100_tr_per_s_sq; // Ramp up/down rate
    intakeLowerMotionMagic.MotionMagicJerk = 1000_tr_per_s_cu;        // Smoothing (optional)

//    configIntakeLowerMotor.Slot0.kS = 0.1; 
//    configIntakeLowerMotor.Slot0.kV = 0.12;
//    configIntakeLowerMotor.Slot0.kP = 0.11;
//    configIntakeLowerMotor.Slot0.kI = 0;
//    configIntakeLowerMotor.Slot0.kD = 0;
    intakeLowerMotor.GetConfigurator().Apply(configIntakeLowerMotor);
 
    // Deploy (Position)
    configDeployMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast; 
    // Slot 0 settings
    configDeployMotor.Slot0.kS = 0.25;  // Add 0.25 V output to overcome static friction
    configDeployMotor.Slot0.kV = 0.012; // A velocity target of 1 rps results in 0.12 V output
    configDeployMotor.Slot0.kA = 0.1;   // An acceleration of 1 rps/s requires 0.01 V output
    configDeployMotor.Slot0.kP = 0.5;   // A position error of 2.5 rotations results in 12 V output
    configDeployMotor.Slot0.kI = 0;     // no output for integrated error
    configDeployMotor.Slot0.kD = 0.1;   // A velocity error of 1 rps results in 0.1 V output   configTurretMotor.Slot0.kP = 1.2;
    // Motion Magic settings
    // SMALL ANGLE configuration settings
    configDeployMotor.MotionMagic.MotionMagicAcceleration = 0_tr_per_s_sq;   // acceleration  (ramp)
    configDeployMotor.MotionMagic.MotionMagicCruiseVelocity = 6.0_tps;       // velocity  (once ramp up)
    configDeployMotor.MotionMagic.MotionMagicExpo_kV = (ctre::unit::volts_per_turn_per_second_t) 0.1; // Speed per unit of voltage (rotations/sec/V)
    configDeployMotor.MotionMagic.MotionMagicExpo_kA = (ctre::unit::volts_per_turn_per_second_squared_t)0.1; // Acceleration per unit of voltage (rotations/sec^2/V) 
    deployMotor.GetConfigurator().Apply(configDeployMotor); 
    deployMotor.GetConfigurator().Apply(configDeployMotor.MotionMagic); 


    // Ensure Turret Motor is in ZERO position.
    zeroizeTurretPosition ();
    // Ensure Hood Motor is in ZERO position.
    zeroizeHoodPosition ();
    // Ensure Intake Deploy Motor is in ZERO position.
    zeroizeIntakePosition ();

    // RAISE the Intake to get it away from the Turret 
    raiseIntake();   

    frc::SmartDashboard::PutNumber("ShooterRPS", m_shooterRPS);
    frc::SmartDashboard::PutNumber("FeederRPS",  m_feederRPS);
    frc::SmartDashboard::PutNumber("SpindexerRPS", m_spindexerRPS);
    frc::SmartDashboard::PutNumber("IntakeUpperRPS", m_intakeUpperRPS);
    frc::SmartDashboard::PutNumber("IntakeLowerRPS", m_intakeLowerRPS);

    // Force Commands to go out to each motor
    m_isTurretClassConfigComplete = true;

    frc2::cmd::Wait(0.25_s);  // Wait for Intake to raise up, before enbling Turret

    // Turret is enabled at the start.
    isTurretActive = true;
    // Shot Table is enabled at the start
    m_isShotTableEnabled = true;

    m_lastStartingPose = frc::Pose2d {0_m, 0_m, frc::Rotation2d(0_deg)};   
    m_startPoseChooser.SetDefaultOption("Face Trench-Outpost", frc::Pose2d {3.64_m, 0.64_m, frc::Rotation2d(0_deg)});
    m_startPoseChooser.AddOption("Face Depot", frc::Pose2d {3.64_m, 6.13_m, frc::Rotation2d(180_deg)}); 
    m_startPoseChooser.AddOption("Face Trench-Depot", frc::Pose2d {3.64_m, 7.5_m, frc::Rotation2d(0_deg)});
    frc::SmartDashboard::PutData("StartPose", &m_startPoseChooser);

}  
 
void Turret::SimulationPeriodic() {}

void Turret::Periodic() {

    // Get the starting Robot Pose and reset the Robot for that Pose
    m_startingPose = m_startPoseChooser.GetSelected();
    if ( (m_startingPose.X().value() != m_lastStartingPose.X().value()) ||
         (m_startingPose.Y().value() != m_lastStartingPose.Y().value()) ||
         ((m_startingPose.Rotation().Degrees().value() != m_lastStartingPose.Rotation().Degrees().value()))) {
       // Identify the Pose read from the dashboard
       m_lastStartingPose = m_startingPose;
       m_newRobotPose = m_startingPose;
       if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
             m_newRobotPose = flipPose(m_newRobotPose);
       }
       frc::SmartDashboard::PutNumber("StartPoseX", m_newRobotPose.X().value());  
       frc::SmartDashboard::PutNumber("StartPoseY", m_newRobotPose.Y().value());  
       frc::SmartDashboard::PutNumber("StartPoseA", m_newRobotPose.Rotation().Degrees().value());  

       // Request Swerve Odometry to reset the Robot Pose 
       m_drivePtr->m_newRobotStartPose = m_newRobotPose;
       m_drivePtr->m_isNewRobotStartPoseResetSelected = true;

    } 

    if (isTopEndReversed) {
       frc::SmartDashboard::PutString("TopEnd REV", "YES"); 
    } else {
       frc::SmartDashboard::PutString("TopEnd REV", "NO"); 
    }
    if (isIntakeReversed) {
       frc::SmartDashboard::PutString("Intake REV", "YES"); 
    } else {
       frc::SmartDashboard::PutString("Intake REV", "NO"); 
    }
    if (isManualOperation) {
       frc::SmartDashboard::PutString("Manual Ctrl", "YES"); 
    } else {
       frc::SmartDashboard::PutString("Manual Ctrl", "NO"); 
    }   
    if (isCamerasInUse) {
       frc::SmartDashboard::PutString("Cameras Used", "YES"); 
      
    } else {
       frc::SmartDashboard::PutString("Cameras Used", "NO"); 
    }   


    // Control Turret Subsystem Opoeration from Smart Dashboard
    string cmdAction = m_cmdActionChooser.GetSelected();
    if (cmdAction != m_lastCmdAction) {

        if (cmdAction == "TurretON") {
            startTurret();
        } 
        else if (cmdAction  == "TurretOFF") {
            stopTurret();
        } 
        else if (cmdAction  == "HoodON") {
            startHood();
        } 
        else if (cmdAction  == "HoodOFF") {
            stopHood();
        } 
        else if (cmdAction  == "ShooterON") {
            startShooter();
        } 
        else if (cmdAction  == "ShooterOFF") {
            stopShooter();
        }
        else if (cmdAction  == "FeederON") {
            startFeeder();
        } 
        else if (cmdAction  == "FeederOFF") {
            stopFeeder();
        }   
        else if (cmdAction  == "SpindexerON") {
            startSpindexer();
        } 
        else if (cmdAction  == "SpindexerOFF") {
            stopSpindexer();
        }
        else if (cmdAction  == "IntakeENABLE") {
            isIntakeActive = true;
        }        
        else if (cmdAction  == "IntakeDISABLE") {
            isIntakeActive = false;
        }                
        else if (cmdAction  == "IntakeDEPLOY") {
            deployIntake();
        }    
        else if (cmdAction  == "IntakeRAISE") {
            raiseIntake();
        }            
        else if (cmdAction  == "IntakeRETRACT") {
            retractIntake();
        }  
        else if (cmdAction  == "TopEndON") {
            enableTopEndOperation();
        } 
        else if (cmdAction  == "TopEndOFF") {
            disableTopEndOperation();
        } 
        else if (cmdAction  == "ShotTableON") {
            m_isShotTableEnabled = true;
        } 
        else if (cmdAction  == "ShotTableOFF") {
            m_isShotTableEnabled = false;
        } 
         else if (cmdAction  == "CompON") {
            m_isCompensationEnabled = true;
        } 
        else if (cmdAction  == "CompOFF") {
            m_isCompensationEnabled = false;
        }    
        // Indicate the last command action, so it is NOT done again.
        m_lastCmdAction = cmdAction; 
    }

    // Get current settings, in case manual operation in effect
    m_turretAngle = frc::SmartDashboard::GetNumber("TurretANGLE", 0.0);
    m_hoodAngle = frc::SmartDashboard::GetNumber("HoodANGLE", 0.0);
    m_shooterRPS = frc::SmartDashboard::GetNumber("ShooterRPS", 10.0);
    m_feederRPS = frc::SmartDashboard::GetNumber("FeederRPS", 3.4);
    m_spindexerRPS = frc::SmartDashboard::GetNumber("SpindexerRPS", 3.4);
    m_intakeUpperRPS = frc::SmartDashboard::GetNumber("IntakeUpperRPS", 20.0);
    m_intakeLowerRPS = frc::SmartDashboard::GetNumber("IntakeLowerRPS", 20.0);


    // ****************************************
    // Determine Turret Angle and Distance to the Target
    // (Since Turret Target has been Identified) 
    if (m_isTurretClassConfigComplete)
    {
        // Get the current robot pose
        frc::Pose2d  m_pose = m_drivePtr->getPose();

        if (m_isManualTgtSelection || isManualOperation) 
        {
            // MANUAL Target Selection
            string tgtSelection = m_shooterTgtChooser.GetSelected();
            if (tgtSelection != m_lastTgtSelection) {
                if (tgtSelection == "BLUEHub") {
                    setTurretTarget (m_BLUE_TgtHub);
                }
                else if (tgtSelection == "BLUEOutpost") {
                    setTurretTarget (m_BLUE_Outpost);
                }
                else if (tgtSelection == "BLUEDepot") {
                    setTurretTarget (m_BLUE_Depot);
                }
                else if (tgtSelection == "REDHub") {
                    setTurretTarget (m_RED_TgtHub);
                }
                else if (tgtSelection == "REDOutpost") {
                    setTurretTarget (m_RED_Outpost);
                }
                else if (tgtSelection == "REDDepot") {
                    setTurretTarget (m_RED_Depot);
                }               
            }
            // Manual Tgt Selection
        }
        else if (!isManualOperation) {
            // ****************************************
            // AUTO Compute Target Selection based upon field location
            // When BLUE Alliance, we can be on the Alliance Side OR the Neutral Zone
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)    
            {
                // Check if we are in the alliance zone
                if (m_pose.X().value() < 4.6) {
                   setTurretTarget (m_BLUE_TgtHub);
                }
                else {
                   if (m_pose.Y().value() < 4.0) {
                      setTurretTarget (m_BLUE_Outpost);
                   } else {
                      setTurretTarget (m_BLUE_Depot);
                   }
                }      
            } 
            // When RED Alliance, we can be on the Alliance Side OR the Neutral Zone
            else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) 
            {  
                // Check if we are in the alliance zone
                if (m_pose.X().value() > 12.0) {
                   setTurretTarget (m_RED_TgtHub);
                }
                else {
                   if (m_pose.Y().value() < 4.0) {
                      setTurretTarget (m_RED_Depot);
                   } else {
                      setTurretTarget (m_RED_Outpost );
                   }
                }      
            }
        }  //Auto Selection
        // ****************************************

        // Determine the Turret Pose (Relative to the Robot Pose)
        m_turretPose = frc::Pose2d{m_pose.Translation() + m_turretTranslation.RotateBy(m_pose.Rotation()), m_pose.Rotation() + frc::Rotation2d{180_deg}};

        // Compute Shooting solution (stationary) Turret Angle and Distance
        // The field relative angle between the turret and the target (zero degrees is the x-axis)
        // The angle should always be -90 to +90
        double turretToTargetAngle = computeTurretToTgtAngleInDegrees(m_turretPose,
                                                                      m_turretTarget );

        if (isManualOperation) {
            m_turretToTargetDistance = frc::SmartDashboard::GetNumber("TurretTgtDistance", 1.0);
        }
        else {
            m_turretToTargetDistance = computeDistanceInMeters(m_turretPose.X().value(),
                                                               m_turretPose.Y().value(),
                                                               m_turretTarget.X().value(),
                                                               m_turretTarget.Y().value());
        }

        // Convert Distance to SHOT SOLUTION MAP table entry index
        m_shotTableIndex = (int) ((m_turretToTargetDistance / 3.0) * 10);     // 1/3 meter steps
        if (m_shotTableIndex <  3) m_shotTableIndex = 3;
        if (m_shotTableIndex > 24) m_shotTableIndex = 24;

        // get shooter velocity before robot velocity compensation
        double shooterVelocity = getShooterRPS(m_shotTableIndex) * 2 * 0.0254 * M_PI;
        // get shooter RPS before robot vlocity compensation
        double shooterRPS = getShooterRPS(m_shotTableIndex);
        // get hood angle before robot velocity compensation
        double hoodAngle = getHoodAngle(m_shotTableIndex);

        
        // Save Shot Solution (Uncompensated) 
        ShotSetpoint shotSolutionRaw  = { turretToTargetAngle, hoodAngle, shooterRPS }; 
   
        // Save Shot Solution (Compensate for Robot speed/heading)
        ShotSetpoint shotSolutionComp = getVelocityCompensatedShotSetpoint(turretToTargetAngle, hoodAngle, shooterRPS);

        // output values to SmartDashboard for sanity check
        frc::SmartDashboard::PutNumber("hoodAngleComp", shotSolutionComp.hood_AngleDegrees);
        frc::SmartDashboard::PutNumber("shooterRPSComp", shotSolutionComp.shooter_RPS);
        frc::SmartDashboard::PutNumber("turretToTgtAngleComp", shotSolutionComp.turret_AngleDegrees);

        // **************************************** 
        frc::SmartDashboard::PutNumber("TurretTargetX", m_turretTarget.X().value());  
        frc::SmartDashboard::PutNumber("TurretTargetY", m_turretTarget.Y().value());  
 
        frc::SmartDashboard::PutNumber("TurretPoseX", m_turretPose.X().value());  
        frc::SmartDashboard::PutNumber("TurretPoseY", m_turretPose.Y().value());  
 
        frc::SmartDashboard::PutNumber("TurretTgtAngle", turretToTargetAngle);  
        if (!isManualOperation) {
            frc::SmartDashboard::PutNumber("TurretTgtDistance", m_turretToTargetDistance);  
        }
        // ****************************************



        // ************************************************
        // Adjust the Turret Motors for Proper Operation
        // SET THE TURRET (if enabled) for proper shooting direction to the target
        if (isTurretActive) {

            // Determine TURRET ANGLE
            double turretAngle = m_turretAngle;  // TESTING - Use SmartDashboard variable
            if (m_isShotTableEnabled && !isManualOperation) {
                turretAngle = shotSolutionRaw.turret_AngleDegrees;    // LIVE OPERATION
            }
            if (m_isCompensationEnabled && !isManualOperation) {
                turretAngle = shotSolutionComp.turret_AngleDegrees;   // LIVE OPERATION
            }
 
            // Limit Turret Angle changes to >= 0.2 degrees
            if (units::math::abs<units::degree_t>((units::degree_t)(turretAngle - m_lastCmdTurretAngle)) >= 0.2_deg) 
            {           
               // Point Turret to the Requested Angle
               setTurretPosition(turretAngle);
               m_lastCmdTurretAngle = turretAngle;
            }       
        }
        // SET THE HOOD (if enabled) for proper shooting angle to the target
        if (isHoodActive) {

            // Determine HOOD ANGLE 
            double hoodAngle = m_hoodAngle; // TESTING - Use SmartDashboard variable
            if (m_isShotTableEnabled && !isManualOperation) {
                 hoodAngle = shotSolutionRaw.hood_AngleDegrees;    // LIVE OPERATION
            }
            if (m_isCompensationEnabled && !isManualOperation) {
                 hoodAngle = shotSolutionComp.hood_AngleDegrees;   // LIVE OPERATION
            }           

            // Limit Hood Angle changes to >= 0.2 degrees
            if (units::math::abs<units::degree_t>((units::degree_t)(hoodAngle - m_lastCmdHoodAngle)) >= 0.2_deg)      
            {
               setHoodPosition (hoodAngle);
               m_lastCmdHoodAngle = hoodAngle;
            }
        }
        // SET THE SHOOTER (if enabled) for proper shooting speed to the target
        if (isShooterActive) {
            
            // Determine SHOOTER RPS 
            double shooterRPS = m_shooterRPS; // TESTING - Use SmartDashboard variable
            if ((m_isShotTableEnabled) && (!isManualOperation)) {
                 shooterRPS = shotSolutionRaw.shooter_RPS;    // LIVE OPERATION
            }      

            // Limit Shooter changes to >= 0.2 RPS
            if (units::math::abs<units::degree_t>((units::degree_t)(shooterRPS - m_lastCmdShooterRPS)) >= 0.2_deg) 
            {       
                setShooterRPS (shooterRPS);
                m_lastCmdShooterRPS = shooterRPS;
            }
        }
        // SET THE FEEDER (if enabled) for proper feeding to the shooter
        if (isFeederActive) { 

            // Get FEEDER rps fom distance table lookup
            double feederRPS = m_feederRPS;
            if (m_isShotTableEnabled && !isManualOperation) {
                 feederRPS = getFeederRPS(m_shotTableIndex);;    // LIVE OPERATION
            }

            // Limit Feeder changes to >= 0.2 RPS
            if ((units::math::abs<units::degree_t>((units::degree_t)(feederRPS - m_lastCmdFeederRPS)) >= 0.2_deg) ||
                (isFeederReversed != isTopEndReversed))
            {       
                setFeederRPS (feederRPS);
                m_lastCmdFeederRPS = feederRPS;
            }
        }
 
        // SET THE SPINDEXER (if enabled) for proper feeding into the shooter
        if (isSpindexerActive) {  
            
            // Get SPINDEXER rps fom distance table lookup
            double spindexerRPS = m_spindexerRPS;
            if (m_isShotTableEnabled && !isManualOperation) {
                 spindexerRPS = getSpindexerRPS(m_shotTableIndex);    // LIVE OPERATION
            }           

            // Limit Spindexer changes to >= 0.2 RPS
            if ((units::math::abs<units::degree_t>((units::degree_t)(spindexerRPS - m_lastCmdSpindexerRPS)) >= 0.2_deg) ||
                (isSpindexerReversed != isTopEndReversed))
            {       
                setSpindexerRPS (spindexerRPS);
                m_lastCmdSpindexerRPS = spindexerRPS;
            }            
        }

        // SET THE INTAKE (if enabled) for proper intake into the Spindexer
        if (isIntakeActive) {         
            setIntakeRPS ();
        }      
    }
    // ****************************************       

}

//
// MOTOR CONTROL ROUTINES
//
// *** TURRET ***
 void Turret::setTurretPosition (double angle) {

    // NOTE: angle is for CCW coordinate system.
    // NOTE: turns for motor need to go the opposite direction
    // NOTE: Therefore, we change the sign of the requested angle 
    double reqAngle = -angle;
    // Ensure angle NOT MORE than maximum
    if (reqAngle > m_MaxTurretAngle) {
        reqAngle = m_MaxTurretAngle;
        if (isTopEndActive) {
            disableTopEndOperation ();
            isTurretDeadZoneDisabled = true;
        }
    }
    // Ensure angle NOT LESS than minimum
    else if (reqAngle < m_MinTurretAngle) {
        reqAngle = m_MinTurretAngle;
        if (isTopEndActive) {
            disableTopEndOperation ();
            isTurretDeadZoneDisabled = true;
        }
    }
    else {
         if (isTurretDeadZoneDisabled){
            isTurretDeadZoneDisabled = false;
            enableTopEndOperation();
         }
    }
    // ***** REFRESH Motor Configuration (AS NEEDED ONLY)
    frc::SmartDashboard::PutNumber("TurretANGLE", angle);
 

    double mechRotations = reqAngle / 360.0;
    double turretTurns = mechRotations * m_turretGearRatio;
    units::angle::turn_t turns = (units::angle::turn_t) turretTurns;

    // Get the motor position (turns) and compare to current commanded position (turns).
    // If the motor is not close to the requested turn position, use the LARGE angle configuration.
    units::angle::turn_t currentPositionTurns = turretMotor.GetPosition().GetValue();  // Actual Position NOW
    double deltaTurns = (double) units::math::abs<units::degree_t>((units::degree_t) ((double) currentPositionTurns - (double) turns));                // Actual minus Requested
    bool isSmallAngle = true;
    if (deltaTurns > 1.4) isSmallAngle = false;

    // ***** TESTING Motor Configuration (AS NEEDED ONLY)
    //frc::SmartDashboard::PutNumber("CurrPosTurns", (double) currentPositionTurns);
    //frc::SmartDashboard::PutNumber("RequestTurns", (double) turns);
    //frc::SmartDashboard::PutNumber("DeltaTurns", (double) deltaTurns);
    //frc::SmartDashboard::PutNumber("SmallAngle", (double) isSmallAngle);
    //frc::SmartDashboard::PutNumber("LastCFGisSmallAngle", (double) m_lastCfgIsSmallAngle);

    // ***** REFRESH Motor Configuration (AS NEEDED ONLY) for SMALL or LARGE angle operation.
    // 1.4 turns equates to about 10 degrees.  Anything greater is considered a large angle.
    // SMALL angle configuration moves the turret more smoothly.
    // LARGE angle configuration moves the turret very quickly, as is needed to point to the target
    if (isSmallAngle && (!m_lastCfgIsSmallAngle))  {
       m_lastCfgIsSmallAngle = true;
       // Refresh config for SMALL angle configuration (was LARGE previously)
       configTurretMotor.MotionMagic.MotionMagicAcceleration = 0_tr_per_s_sq; // acceleration  (ramp)
       configTurretMotor.MotionMagic.MotionMagicCruiseVelocity = 6_tps;       // velocity  (once ramp up)
       configTurretMotor.MotionMagic.MotionMagicExpo_kV = (ctre::unit::volts_per_turn_per_second_t) 0.12; // Speed per unit of voltage (rotations/sec/V)
       configTurretMotor.MotionMagic.MotionMagicExpo_kA = (ctre::unit::volts_per_turn_per_second_squared_t)0.1; // Acceleration per unit of voltage (rotations/sec^2/V) 
       turretMotor.GetConfigurator().Apply(configTurretMotor.MotionMagic);
    } 
    else if (!isSmallAngle && m_lastCfgIsSmallAngle) {
       m_lastCfgIsSmallAngle = false;
       // Refresh config for LARGE angle configuration (was SMALL previously)
       configTurretMotor.MotionMagic.MotionMagicAcceleration = 11000_tr_per_s_sq; // acceleration  (ramp)
       configTurretMotor.MotionMagic.MotionMagicCruiseVelocity = 16000_tps;       // velocity  (once ramp up)
       configTurretMotor.MotionMagic.MotionMagicExpo_kV = (ctre::unit::volts_per_turn_per_second_t) 0.003; // Speed per unit of voltage (rotations/sec/V)
       configTurretMotor.MotionMagic.MotionMagicExpo_kA = (ctre::unit::volts_per_turn_per_second_squared_t)0.02; // Acceleration per unit of voltage (rotations/sec^2/V)
       turretMotor.GetConfigurator().Apply(configTurretMotor.MotionMagic);       
    }

    ctre::phoenix6::controls::MotionMagicExpoVoltage m_turretRequest{0_tr};
    // Move the turret the desired number of turns, matching the requested angle
    turretMotor.SetControl(m_turretRequest.WithPosition(turns)); 
 }
    
 void Turret::zeroizeTurretPosition () {
    // ASSUMES THE TURRET HAS BEEN MANUALLY ALIGNED, FACING 180 degrees FORWARD, REVERSE OF ROBOT FRONT
    turretMotor.SetPosition ((units::angle::turn_t) 0.0);
    isTurretActive = true;
 }

void Turret::startTurret () {
    m_lastCmdTurretAngle -= 1;  // Force Turret to be newly commanded
    isTurretActive = true;
}

void Turret::stopTurret () {
    isTurretActive = false;
}


double Turret::getHoodAngle (int shotTableIndex) {
    double hoodAngle = 0;

    if ((!m_isShotTableEnabled) || (isManualOperation)){
        hoodAngle = m_hoodAngle;  // Use SmartDashboard variable
    } else {
        ShotSolutionEntry shotSolution = m_shotSolutionMap[shotTableIndex];      
        hoodAngle = shotSolution.hood_ANGLE;
    }
    frc::SmartDashboard::PutNumber("HoodANGLE", hoodAngle);

    return (hoodAngle);
}

void Turret::setHoodPosition (double angle) {
    double reqAngle = angle;
    // Ensure angle NOT MORE than maximum
    if (reqAngle > m_MaxHoodAngle) {
        reqAngle = m_MaxHoodAngle;
    }
    // Ensure angle NOT LESS than minimum
    if (reqAngle < m_MinHoodAngle) {
        reqAngle = m_MinHoodAngle;
    }
    // Subtract MIN from angle, to match zeroized when angle is = m_MinHoodAngle
    reqAngle = reqAngle - m_MinHoodAngle;

    double mechRotations = reqAngle / 360.0;
    m_hoodTurns = mechRotations * m_hoodGearRatio;
    units::angle::turn_t turns = (units::angle::turn_t) m_hoodTurns;
 
    ctre::phoenix6::controls::PositionVoltage hoodRequest = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0); 
    hoodMotor.SetControl(hoodRequest.WithPosition(turns));   
}
    
void Turret::zeroizeHoodPosition () {
    // ASSUMES THE HOOD IS FULLY LOWERED (NOT EXTENDED)
    hoodMotor.SetPosition ((units::angle::turn_t) 0.0);
    m_lastCmdHoodAngle = 10; 
    isHoodActive = true;
}

void Turret::startHood () {
    isHoodActive = true;
}

void Turret::stopHood () {
    // Disable the Hood and Stow (fully retract to allow Trench passage)
    ctre::phoenix6::controls::PositionVoltage hoodRequest = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);   
    hoodMotor.SetControl(hoodRequest.WithPosition(0_tr));  // Hood does NOT fully retract immediatly at zeroized point 
    m_lastCmdHoodAngle = 10; 
    isHoodActive = false;
}



// *** SHOOTER ***
double Turret::getShooterRPS (int shotTableIndex) {
    double shooterRPS = 0;
   
    if ((!m_isShotTableEnabled) || (isManualOperation)) {
        shooterRPS = m_shooterRPS;  // Use SmartDashboard variable
    } else {
        ShotSolutionEntry shotSolution = m_shotSolutionMap[shotTableIndex];      
        shooterRPS = shotSolution.shooter_RPS;
    }
    frc::SmartDashboard::PutNumber("ShooterRPS", shooterRPS);

    return (shooterRPS);
}


void Turret::setShooterRPS (double rps) {
    // Set the motor speeds (FORWARD and REVERSE)
    ctre::phoenix6::controls::VelocityVoltage m_forwardRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
    ctre::phoenix6::controls::VelocityVoltage m_reverseRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

    shooterForwardMotor.SetControl(m_forwardRequest.WithVelocity((units::angular_velocity::turns_per_second_t) rps).WithFeedForward(0.5_V));
    shooterReverseMotor.SetControl(m_reverseRequest.WithVelocity((units::angular_velocity::turns_per_second_t) -rps).WithFeedForward(0.5_V));
}

void Turret::startShooter () {
    isShooterActive = true;
}

void Turret::stopShooter () {
    isShooterActive = false;
    shooterForwardMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    shooterReverseMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});

    shooterForwardMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    shooterReverseMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    m_lastCmdShooterRPS = 0; 
}

// *** FEEDER ***
// *** SHOOTER ***
double Turret::getFeederRPS (int shotTableIndex) {
    double feederRPS = 0;
   
    if (!m_isShotTableEnabled) {
        feederRPS = m_feederRPS;   // Use SmartDashboard variable
    } else {
        ShotSolutionEntry shotSolution = m_shotSolutionMap[shotTableIndex];      
        feederRPS = shotSolution.feeder_RPS;
    }
    frc::SmartDashboard::PutNumber("FeederRPS", feederRPS);

    return (feederRPS);
}

void Turret::setFeederRPS (double feederRPS) {
    // Set the motor speeds (DUAL and SINGLE)
    ctre::phoenix6::controls::MotionMagicVelocityVoltage m_dualRequest{(units::angular_velocity::turns_per_second_t) 0}; 
    ctre::phoenix6::controls::MotionMagicVelocityVoltage m_singleRequest{(units::angular_velocity::turns_per_second_t) 0};   
    //ctre::phoenix6::controls::VelocityVoltage m_dualRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
    //ctre::phoenix6::controls::VelocityVoltage m_singleRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

    // Check if reversal is Requested
    if (isTopEndReversed) {
        feederRPS = -feederRPS;
        isFeederReversed = true;
    } else {
        isFeederReversed = false;
    }

    feederDualMotor.SetControl(m_dualRequest.WithVelocity(feederRPS * 1_tps));
    feederSingleMotor.SetControl(m_singleRequest.WithVelocity(feederRPS * 1_tps));
}

void Turret::startFeeder () {
    isFeederActive = true;
}

void Turret::stopFeeder () {
    isFeederActive = false;
    feederDualMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    feederSingleMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});

    feederDualMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    feederSingleMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    m_lastCmdFeederRPS = 0; 
}

// *** SPINDEXER ***
double Turret::getSpindexerRPS (int shotTableIndex) {
    double spindexerRPS = 0;

    if (!m_isShotTableEnabled) {
        spindexerRPS = m_spindexerRPS;  // Use SmartDashboard variable
    } else {
        ShotSolutionEntry shotSolution = m_shotSolutionMap[shotTableIndex];      
        spindexerRPS = shotSolution.spindexer_RPS;
    }
    frc::SmartDashboard::PutNumber("SpindexerRPS", spindexerRPS);

    return (spindexerRPS);
}

void Turret::setSpindexerRPS (double spindexerRPS) {

    // Check if reversal is Requested
    if (isTopEndReversed) {
        spindexerRPS = -spindexerRPS;
        isSpindexerReversed = true;
    } else {
        isSpindexerReversed = false;
    }
   
    // Set the motor speed
    ctre::phoenix6::controls::MotionMagicVelocityVoltage m_spindexerRequest{(units::angular_velocity::turns_per_second_t) 0}; 
    spindexerMotor.SetControl(m_spindexerRequest.WithVelocity(-(spindexerRPS * m_spindexerGearRatio) * 1_tps));
}

void Turret::startSpindexer () {
    isSpindexerActive = true;
}

void Turret::stopSpindexer () {
    isSpindexerActive = false;

    spindexerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    spindexerMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    m_lastCmdSpindexerRPS = 0; 
}

// *** INTAKE ***
void Turret::setIntakeRPS () {
    // Set the motor speeds (FORWARD and REVERSE)
    ctre::phoenix6::controls::MotionMagicVelocityVoltage m_upperRequest{(units::angular_velocity::turns_per_second_t) 0}; 
    ctre::phoenix6::controls::MotionMagicVelocityVoltage m_lowerRequest{(units::angular_velocity::turns_per_second_t) 0};   
 //   ctre::phoenix6::controls::VelocityVoltage m_upperRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
 //   ctre::phoenix6::controls::VelocityVoltage m_lowerRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

    // Check if reversal is Requested
    double intakeUpperRPS = m_intakeUpperRPS;
    if (isIntakeReversed) intakeUpperRPS = -intakeUpperRPS;
    double intakeLowerRPS = m_intakeLowerRPS;
    if (isIntakeReversed) intakeLowerRPS = -intakeLowerRPS;

    intakeUpperMotor.SetControl(m_upperRequest.WithVelocity((units::angular_velocity::turns_per_second_t) (intakeUpperRPS)));
    intakeLowerMotor.SetControl(m_lowerRequest.WithVelocity((units::angular_velocity::turns_per_second_t) -(intakeLowerRPS)));
//    intakeUpperMotor.SetControl(m_upperRequest.WithVelocity((units::angular_velocity::turns_per_second_t) (intakeUpperRPS)).WithFeedForward(0.5_V));
//    intakeLowerMotor.SetControl(m_lowerRequest.WithVelocity((units::angular_velocity::turns_per_second_t) -(intakeLowerRPS)).WithFeedForward(0.5_V));
}

void Turret::zeroizeIntakePosition () {
    // ASSUMES THE INTAKE IS FULLY RETRACTED (NOT EXTENDED)
    deployMotor.SetPosition ((units::angle::turn_t) 0.0);
 
    isIntakeActive = false;
    isIntakeDeployed = false;
}

void Turret::deployIntake () {

    // deploy (fully extend)
    ctre::phoenix6::controls::PositionVoltage deployRequest = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0); 
    deployMotor.SetControl(deployRequest.WithPosition( (units::angle::turn_t) m_intakeDeployPosition)); 

    isIntakeActive = true;
    isIntakeDeployed = true;  
}

void Turret::retractIntake () {
    isIntakeActive = false;
    isIntakeDeployed = false;

    // Disable the Intake and Undeploy (fully retract)
    // Stop the Intake Upper Motor
    intakeUpperMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    intakeUpperMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    // Stop the Intake Lower Motor
    intakeLowerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    intakeLowerMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
   
    // Undeploy (fully retract)
    ctre::phoenix6::controls::PositionVoltage retractRequest = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);;
    deployMotor.SetControl(retractRequest.WithPosition( 0_tr));       
}

void Turret::raiseIntake () {
    isIntakeActive = true;      // Leave the Intake Motors running
    isIntakeDeployed = false;

    // Disable the Intake and Raise Up (do not fully retract)
    // Stop the Intake Upper Motor
    intakeUpperMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    intakeUpperMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    // Stop the Intake Lower Motor
    intakeLowerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    intakeLowerMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
   
    // Undeploy (raise off the floor, but not fully retracted)    
    ctre::phoenix6::controls::PositionVoltage raiseRequest = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);;
    deployMotor.SetControl(raiseRequest.WithPosition((units::angle::turn_t) m_intakeRaisePosition));    
}

void Turret::intakeOnOff () {
    isIntakeActive = !isIntakeActive;     // Change Intake Operation On/Off

    // Stop the Intake, when NOT active
    if (!isIntakeActive) {
       // Stop the Intake Upper Motor
       intakeUpperMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
       intakeUpperMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
       // Stop the Intake Lower Motor
       intakeLowerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
       intakeLowerMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    }
}




// ****************************************
void Turret::setTurretTarget (frc::Translation2d theShooterTarget) {
    // Set the Target for the Turret
    m_turretTarget = {(units::length::meter_t) theShooterTarget.X().value(),
                      (units::length::meter_t) theShooterTarget.Y().value()};
    m_turretTargetSet = true;
}

frc::Translation2d Turret::getTurretTarget () {
    // Set the Target for the Turret
    return (m_turretTarget);
}
// ****************************************

frc::Pose2d Turret::flipPose(const frc::Pose2d& pose) {
    auto flippedX = MainConst::kFieldLength - pose.X();
    auto flippedY = MainConst::kFieldWidth  - pose.Y();  
    // Rotate heading by 180 degrees
    frc::Rotation2d flippedRotation = pose.Rotation() + frc::Rotation2d(180_deg);
    frc::Pose2d newPose = frc::Pose2d{flippedX, flippedY, flippedRotation};
    return newPose;
}

// ****************************************
double Turret::computeDistanceInMeters(double x1, double y1, double x2, double y2) {
    double x_diff = x2 - x1;
    double y_diff = y2 - y1;
    // Apply the distance formula
    double distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    return distance;
}

// NOTE: robotPose is ASSUMED to be the center of the robot.
//       The robotPose must be converted to turretPose.
//       It is ASSUMED that robotPose has ALREADY accounted for camera(s) placement(s) 
//       withinn the odometry processing.
//    
//       The turretPose must use robotPose and account for turret placement.
//       (Turret Offset from robot center AND rotation on the field due to robot heading.)
//
// NOTE: This routine returns the turret shooting solution angle.
//       The angle is +/- 180 degrees from the turret perspective.
//       As a reminder, the turret faces 180 degrees OPPOSITE the robot front.
double Turret::computeTurretToTgtAngleInDegrees(frc::Pose2d turretPose, frc::Translation2d turretTarget )
{
    double pi_val = 4.0 * std::atan(1.0);       // Compute PI to many digits, atan (1 radian) = PI/4
    double RadiansToDegrees = 180.0 / pi_val;   // Radians to Degrees Conversion Factor
    //double DegreesToRadians = pi_val / 180.0; // Degrees to Radians Conversion Factor

    // Compute the Angle from the Turret X,Y Position to the Target X,Y Position
    double delta_X = (double) (turretTarget.X() - turretPose.X());
    double delta_Y = (double) (turretTarget.Y() - turretPose.Y());
    double robotToTgtAngleRadians = atan2(delta_Y, delta_X);           // Radians
    double robotToTgtAngleDegrees = robotToTgtAngleRadians * RadiansToDegrees;  // Degrees

    frc::SmartDashboard::PutNumber("FieldTgtAngle", robotToTgtAngleDegrees); 

    // ******
    // Determine the non-compensated angle the Turret should be set to point to the target
    double turretAngleDegrees = robotToTgtAngleDegrees - m_turretPose.Rotation().Degrees().value();
    double turretAngleChange = turretAngleDegrees - m_lastTurretAngle;
    am::limitDegrees(turretAngleChange);
    turretAngleDegrees = m_lastTurretAngle + turretAngleChange;
    while (turretAngleDegrees > MAX_TURRET_ROTATION_ANGLE) {
        turretAngleDegrees -= 360;
    }
    while (turretAngleDegrees < -MAX_TURRET_ROTATION_ANGLE) {
         turretAngleDegrees += 360;
    }
    m_lastTurretAngle = turretAngleDegrees;
    // *****

    return (m_lastTurretAngle);
}

// make a robot velocity compensated vector for determining the turret angle, hood angle, and shooter velocity with compensation for the robot velocity.
Turret::ShotSetpoint Turret::getVelocityCompensatedShotSetpoint(double robotToTargetAngle, double shooterRPS, double hoodAngleDegrees) {

    // get shooter velocity before robot velocity compensation
    double shooterVelocityMPS = shooterRPS * 2 * 0.0254 * M_PI;

    double hoodAngleRadians = hoodAngleDegrees * M_PI / 180;
    double robotToTargetAngleRadians = robotToTargetAngle * M_PI / 180;
    auto shooterHorizontalVelocityMPS = shooterVelocityMPS * sin(hoodAngleRadians);
    auto shooterVerticalVelocityMPS = shooterVelocityMPS * cos(hoodAngleRadians);
    auto shooterVelocityVectorMPS = Eigen::Vector3d(shooterHorizontalVelocityMPS * cos(robotToTargetAngleRadians),
                                           shooterHorizontalVelocityMPS * sin(robotToTargetAngleRadians),
                                           shooterVerticalVelocityMPS
                                           );
    // get robot chassis speed in field space
    auto robotSpeed = m_drivePtr->getFieldRelativeSpeeds();
    // convert to vector in 3D space and subtract to compensate
    shooterVelocityVectorMPS -= Eigen::Vector3d(robotSpeed.vx.value(), robotSpeed.vy.value(), 0);
    double newHorizontalVelocity = hypot(shooterVelocityVectorMPS[0], shooterVelocityVectorMPS[1]);
    double newVerticalVelocity = shooterVelocityVectorMPS[2];
    double newHoodAngleDegrees = atan2(newHorizontalVelocity, newVerticalVelocity) * 180 / M_PI;
    double newTurretAngleDegreesComp = atan2(shooterVelocityVectorMPS[1],shooterVelocityVectorMPS[0]);

     frc::SmartDashboard::PutNumber("turretAngleComp", newTurretAngleDegreesComp);

    double newTurretAngleDegrees  = robotToTargetAngle + newTurretAngleDegreesComp;
    am::limitDegrees(newTurretAngleDegrees);  

    double newShooterRPS = shooterVelocityVectorMPS.norm() / (2 * 0.0254 * M_PI);
    if (newHoodAngleDegrees < 10) {newHoodAngleDegrees = 10;}
    if (newHoodAngleDegrees > 50) {newHoodAngleDegrees = 50;}
    
    return ShotSetpoint{newTurretAngleDegrees, newHoodAngleDegrees, newShooterRPS};
}

// ****************************************

void Turret::enableTurretOperation ()     { startTurret();    }
void Turret::enableHoodOperation ()       { startHood();      }
void Turret::enableShooterOperation ()    { startShooter();   }
void Turret::enableFeederOperation ()     { startFeeder();    }
void Turret::enableSpindexerOperation ()  { startSpindexer(); }
void Turret::deployIntakeOperation ()     { deployIntake();   }
void Turret::raiseIntakeOperation ()      { raiseIntake();    }
void Turret::enableTopEndOperation ()     { startHood(); startShooter(); startHood(); startFeeder(); startSpindexer(); isTopEndActive = true; }

void Turret::disableTurretOperation ()    { stopTurret();     }
void Turret::disableHoodOperation ()      { stopHood();       }
void Turret::disableShooterOperation ()   { stopShooter();    }
void Turret::disableFeederOperation ()    { stopFeeder();     }
void Turret::disableSpindexerOperation () { stopSpindexer();  }
void Turret::retractIntakeOperation ()    { retractIntake();  }
void Turret::reverseIntakeOperation ()    { isIntakeReversed = !isIntakeReversed; } 
void Turret::disableTopEndOperation ()    { stopSpindexer(); stopSpindexer(); stopFeeder(); stopHood(); stopShooter(); isTopEndActive = false; }
void Turret::reverseTopEndOperation ()    { isTopEndReversed  = !isTopEndReversed; } 
void Turret::manualOperation ()           { isManualOperation = !isManualOperation; }
void Turret::intakeOnOffOperation ()      { intakeOnOff(); }
void Turret::cameraOperation()            { 
    isCamerasInUse = !isCamerasInUse; 
    if (isCamerasInUse) {
        m_drivePtr->m_isCamerasUsedForOdometry = true;
    } else {
        m_drivePtr->m_isCamerasUsedForOdometry = false;
    };
}

frc2::CommandPtr Turret::cmdOnTurret()  {
    return RunOnce([this] { enableTurretOperation(); }).WithName("Enable Turret Operation");
}

frc2::CommandPtr Turret::cmdOnHood() {
   return RunOnce([this] { enableHoodOperation(); }).WithName("Enable Hood Operation");
}

frc2::CommandPtr Turret::cmdOnShooter() {
   return RunOnce([this] { enableShooterOperation(); }).WithName("Enable Shooter Operation");
}

frc2::CommandPtr Turret::cmdOnFeeder() {
   return RunOnce([this] { enableFeederOperation(); }).WithName("Enable Feeder Operation");
}

frc2::CommandPtr Turret::cmdOnSpindexer() {
   return RunOnce([this] { enableSpindexerOperation(); }).WithName("Enable Spindexer Operation");
}

frc2::CommandPtr Turret::cmdDeployIntake() {
   return RunOnce([this] { deployIntakeOperation(); }).WithName("Deploy Intake Operation");
}

frc2::CommandPtr Turret::cmdRaiseIntake() {
   return RunOnce([this] { raiseIntakeOperation(); }).WithName("Raise Intake Operation");
}

frc2::CommandPtr Turret::cmdOnTopEnd() {
   return RunOnce([this] { enableTopEndOperation(); }).WithName("Enable TopEnd Operation");
}



frc2::CommandPtr Turret::cmdOffTurret()  {
    return RunOnce([this] { disableTurretOperation(); }).WithName("Disable Turret Operation");
}

frc2::CommandPtr Turret::cmdOffHood() {
   return RunOnce([this] { disableHoodOperation(); }).WithName("Disable Hood Operation ");
}

frc2::CommandPtr Turret::cmdOffShooter() {
   return RunOnce([this] { disableShooterOperation(); }).WithName("Disable Shooter Operation");
}

frc2::CommandPtr Turret::cmdOffFeeder() {
   return RunOnce([this] { disableFeederOperation(); }).WithName("Disable Feeder Operation");
}

frc2::CommandPtr Turret::cmdOffSpindexer() {
   return RunOnce([this] { disableSpindexerOperation(); }).WithName("Disable Spindexer Operation");
}

frc2::CommandPtr Turret::cmdRetractIntake() {
   return RunOnce([this] { retractIntakeOperation(); }).WithName("Retract Intake Operation");
}

frc2::CommandPtr Turret::cmdReverseIntake() {
   return RunOnce([this] { reverseIntakeOperation(); }).WithName("Retract Intake Operation");
}

frc2::CommandPtr Turret::cmdOffTopEnd() {
   return RunOnce([this] { disableTopEndOperation(); }).WithName("Disable TopEnd Operation");
}

frc2::CommandPtr Turret::cmdRevTopEnd() {
   return RunOnce([this] { reverseTopEndOperation(); }).WithName("Reverse TopEnd Operation");
}

frc2::CommandPtr Turret::cmdManualOperation() {
   return RunOnce([this] { manualOperation(); }).WithName("Manual Operation");
}

frc2::CommandPtr Turret::cmdIntakeON_OFF() {
   return RunOnce([this] { intakeOnOffOperation(); }).WithName("Intake ON or OFF");
}

frc2::CommandPtr Turret::cmdUseCameras() {
   return RunOnce([this] { cameraOperation(); }).WithName("Cameras ON or OFF");
}

// Turret Destructor
Turret::~Turret() {}
