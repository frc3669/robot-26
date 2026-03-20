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
    // LARGE ANGLE configuration settings
    //configTurretMotor.MotionMagic.MotionMagicAcceleration = 10000_tr_per_s_sq; // acceleration  (ramp)
    //configTurretMotor.MotionMagic.MotionMagicCruiseVelocity = 16000_tps;       // velocity  (once ramp up)
    //configTurretMotor.MotionMagic.MotionMagicExpo_kV = (ctre::unit::volts_per_turn_per_second_t) 0.003; // Speed per unit of voltage (rotations/sec/V)
    //configTurretMotor.MotionMagic.MotionMagicExpo_kA = (ctre::unit::volts_per_turn_per_second_squared_t)0.02; // Acceleration per unit of voltage (rotations/sec^2/V)
 
    // Apply Configuration 
    turretMotor.GetConfigurator().Apply(configTurretMotor);

 
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
    configFeederDualMotor.Slot0.kS = 0.1;   
    configFeederDualMotor.Slot0.kV = 0.12;
    configFeederDualMotor.Slot0.kP = 0.11;
    configFeederDualMotor.Slot0.kI = 0;
    configFeederDualMotor.Slot0.kD = 0;
    feederDualMotor.GetConfigurator().Apply(configFeederDualMotor);
    configFeederSingleMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configFeederSingleMotor.Slot0.kS = 0.1; 
    configFeederSingleMotor.Slot0.kV = 0.12;
    configFeederSingleMotor.Slot0.kP = 0.11;
    configFeederSingleMotor.Slot0.kI = 0;
    configFeederSingleMotor.Slot0.kD = 0;
    feederSingleMotor.GetConfigurator().Apply(configFeederSingleMotor);
 
    // Spindexer Motor (Speed)
    configSpindexerMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configSpindexerMotor.Slot0.kS = 0.1;
    configSpindexerMotor.Slot0.kV = 0.12;
    configSpindexerMotor.Slot0.kP = 0.11;
    configSpindexerMotor.Slot0.kI = 0;
    configSpindexerMotor.Slot0.kD = 0;
    spindexerMotor.GetConfigurator().Apply(configSpindexerMotor);

    // Intake Motors (Intake and Deploy)
    // Intake (Speed)
    configIntakeMotor.Slot0.kP = 0.1;
    configIntakeMotor.Slot0.kI = 0;
    configIntakeMotor.Slot0.kD = 0;
    configIntakeMotor.Slot0.kV = 0.12;
    intakeMotor.GetConfigurator().Apply(configIntakeMotor);
    // Deploy (Position)
    configDeployMotor.Slot0.kP = 0.2;
    configDeployMotor.Slot0.kI = 0.0;
    configDeployMotor.Slot0.kD = 0.1;
    deployMotor.GetConfigurator().Apply(configDeployMotor);
    

    // Ensure Turret Motor is in ZERO position.
    zeroizeTurretPosition ();
    // Ensure Hood Motor is in ZERO position.
    zeroizeHoodPosition ();
    // Ensure Intake Deploy Motor is in ZERO position.
    zeroizeIntakePosition ();

    frc::SmartDashboard::PutNumber("ShooterRPS", m_shooterRPS);
    frc::SmartDashboard::PutNumber("FeederRPS",  m_feederRPS);
    frc::SmartDashboard::PutNumber("SpindexerRPS", m_spindexerRPS);
    frc::SmartDashboard::PutNumber("IntakeRPM", m_intakeRPM);

    // Force Commands to go out to each motor
    m_isTurretClassConfigComplete = true;
}  
 
void Turret::SimulationPeriodic() {}

void Turret::Periodic() {

     // ****************************************
    string tgtSelection = m_shooterTgtChooser.GetSelected();
    if (tgtSelection != m_lastTgtSelection) {
        if (tgtSelection == "BLUEHub") {
            setTurretTarget (m_BLUE_TargetHub);
        }
        else if (tgtSelection == "REDHub") {
            setTurretTarget (m_RED_TargetHub);
        }
        frc::SmartDashboard::PutString("TurretTargetID", tgtSelection);          
         // Indicate the last command action, so it is NOT done again.
        m_lastTgtSelection = tgtSelection;       
    }
    // ****************************************

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
        else if (cmdAction  == "IntakeON") {
            startIntake();
        }    
        else if (cmdAction  == "IntakeOFF") {
            stopIntake();
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
        // Indicate the last command action, so it is NOT done again.
        m_lastCmdAction = cmdAction; 
    }

    // Get current settings, in case manual operation in effect
    m_turretAngle = frc::SmartDashboard::GetNumber("TurretANGLE", 0.0);
    m_hoodAngle = frc::SmartDashboard::GetNumber("HoodANGLE", 0.0);
    m_shooterRPS = frc::SmartDashboard::GetNumber("ShooterRPS", 10.0);
    m_feederRPS = frc::SmartDashboard::GetNumber("FeederRPS", 3.4);
    m_spindexerRPS = frc::SmartDashboard::GetNumber("SpindexerRPS", 3.4);
    m_intakeRPS = frc::SmartDashboard::GetNumber("IntakeRPS", 10.0);



    // ****************************************
    // Determine Turret Angle and Distance to the Target
    // (Since Turret Target has been Identified) 
    if (m_isTurretClassConfigComplete)
    {
        // Get the current robot pose
        frc::Pose2d  m_pose = m_drivePtr->getPose();

        // Determine the Turret Pose (Relative to the Robot Pose)
        m_turretPose = frc::Pose2d{m_pose.Translation()+m_turretTranslation.RotateBy(m_pose.Rotation()), m_pose.Rotation() + frc::Rotation2d{180_deg}};

        // Compute Shooting solution (stationary) Turret Angle and Distance
        // The angle between the turret and the target if the robot is facing 0degrees on the field
        double robotToTargetAngle    = computeRobotToTgtAngleInDegrees(m_turretPose,
                                                             m_turretTarget );
        m_turretTargetDistance = computeDistanceInMeters(m_turretPose.X().value(),
                                                         m_turretPose.Y().value(),
                                                         m_turretTarget.X().value(),
                                                         m_turretTarget.Y().value());

                                                     

        // Convert Distance to SHOT SOLUTION MAP table entry index
        m_shotTableIndex = (int) ((m_turretTargetDistance / 3.0) * 10);     // 1/3 meter steps
        if (m_shotTableIndex <  3) m_shotTableIndex = 3;
        if (m_shotTableIndex > 24) m_shotTableIndex = 24;

        // get shooter velocity before robot velocity compensation
        double shooterVelocity = getShooterRPS(m_shotTableIndex) * 2 * 0.0254 * M_PI;
        // get hood angle before robot velocity compensation
        double hoodAngle = getHoodAngle(m_shotTableIndex);
        
        // compensate for robot velocity
        ShotSetpoint velocityCompensatedShooterSetpoint = getVelocityCompensatedShotSetpoint(robotToTargetAngle, shooterVelocity, hoodAngle);
        // 
        double turretAngle = m_turretPose.Rotation().Degrees().value() - velocityCompensatedShooterSetpoint.robotToTargetAngleDegrees;
        am::limitDegrees(turretAngle);
        // m_turretAngle = turretAngle;
        // m_hoodAngle = velocityCompensatedShooterSetpoint.hoodAngleDegrees;
        // m_shooterRPS = velocityCompensatedShooterSetpoint.shooter_RPS;
        // output values to SmartDashboard for sanity check
        frc::SmartDashboard::PutNumber("turretAngleAfterCompensation", turretAngle);
        frc::SmartDashboard::PutNumber("hoodAngleAfterCompensation", velocityCompensatedShooterSetpoint.hoodAngleDegrees);
        frc::SmartDashboard::PutNumber("shooterRPSAfterCompensation", velocityCompensatedShooterSetpoint.shooter_RPS);



        // ************************************************
        // Adjust the Turret Motors for Proper Operation
        // SET THE TURRET (if enabled) for proper shooting direction to the target
        if (isTurretActive) {

            // Determine turret angle 
            // double turretAngle = m_turretTargetAngle;   // LIVE OPERATION
            double turretAngle = m_turretAngle;            // TESTING - Use SmartDashboard variable

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
 
            double hoodAngle = m_hoodAngle;

            // Limit Hood Angle changes to >= 0.2 degrees
           if (units::math::abs<units::degree_t>((units::degree_t)(hoodAngle - m_lastCmdHoodAngle)) >= 0.2_deg)      
           {
              setHoodPosition (hoodAngle);
              m_lastCmdHoodAngle = hoodAngle;
           }
        }
        // SET THE SHOOTER (if enabled) for proper shooting speed to the target
        if (isShooterActive) {
            
            // Get SHOOTER rps fom distance table lookup
            double shooterRPS = m_shooterRPS;

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
            double feederRPS = getFeederRPS(m_shotTableIndex);

            // Limit Feeder changes to >= 0.2 RPS
            if (units::math::abs<units::degree_t>((units::degree_t)(feederRPS - m_lastCmdFeederRPS)) >= 0.2_deg) 
            {       
                setFeederRPS (feederRPS);
                m_lastCmdFeederRPS = feederRPS;
            }
        }
 
        // SET THE SPINDEXER (if enabled) for proper feeding into the shooter
        if (isSpindexerActive) {  
            
            // Get SPINDEXER rps fom distance table lookup
            double spindexerRPS = getSpindexerRPS(m_shotTableIndex);           

            // Limit Feeder changes to >= 0.2 RPS
            if (units::math::abs<units::degree_t>((units::degree_t)(spindexerRPS - m_lastCmdSpindexerRPS)) >= 0.2_deg) 
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

    // **************************************** 
    frc::SmartDashboard::PutNumber("TurretTargetX", m_turretTarget.X().value());  
    frc::SmartDashboard::PutNumber("TurretTargetY", m_turretTarget.Y().value());  
 
    frc::SmartDashboard::PutNumber("TurretPoseX", m_turretPose.X().value());  
    frc::SmartDashboard::PutNumber("TurretPoseY", m_turretPose.Y().value());    
    frc::SmartDashboard::PutNumber("TurretTgtAngle", m_turretTargetAngle);  
    frc::SmartDashboard::PutNumber("TurretTgtDistance", m_turretTargetDistance);  
    // ****************************************
}

//
// MOTOR CONTROL ROUTINES
//
// *** TURRET ***
 void Turret::setTurretPosition (double angle) {

    // *****
    // Determine whether SMALL or LARGE angle change (> TBD degrees, e.g. 45) for the turret
    bool isSmallAngle = true;
    if (units::math::abs<units::degree_t>((units::degree_t)(angle - m_lastCmdTurretAngle)) >= 45.0_deg) {
        isSmallAngle = false;  // TRUE by default
    }
    // NOTE: Update LAST commanded angle information after information has been used. (See Below)
    // *****

    // NOTE: angle is for CCW coordinate system.
    // NOTE: turns for motor need to go the opposite direction
    // NOTE: Therefore, we change the sign of the requested angle 
    double reqAngle = -angle;
    // Ensure angle NOT MORE than maximum
    if (reqAngle > m_MaxTurretAngle) {
        reqAngle = m_MaxTurretAngle;
    }
    // Ensure angle NOT LESS than minimum
    if (reqAngle < m_MinTurretAngle) {
        reqAngle = m_MinTurretAngle;
    }
 
    double mechRotations = reqAngle / 360.0;
    double turretTurns = mechRotations * m_turretGearRatio;
    units::angle::turn_t turns = (units::angle::turn_t) turretTurns;

    // ***** REFRESH Motor Configuration (AS NEEDED ONLY) for SMALL or LARGE angle operation.
    if (isSmallAngle) {
       // Refresh config for SMALL angle configuration (was LARGE previously)
       configTurretMotor.MotionMagic.MotionMagicAcceleration = 0_tr_per_s_sq; // acceleration  (ramp)
       configTurretMotor.MotionMagic.MotionMagicCruiseVelocity = 6_tps;       // velocity  (once ramp up)
       configTurretMotor.MotionMagic.MotionMagicExpo_kV = (ctre::unit::volts_per_turn_per_second_t) 0.12; // Speed per unit of voltage (rotations/sec/V)
       configTurretMotor.MotionMagic.MotionMagicExpo_kA = (ctre::unit::volts_per_turn_per_second_squared_t)0.1; // Acceleration per unit of voltage (rotations/sec^2/V) 
       ctre::phoenix::StatusCode Sstatus = turretMotor.GetConfigurator().Apply(configTurretMotor.MotionMagic);
       frc::SmartDashboard::PutNumber("S-StatusCode", Sstatus);  
       frc::SmartDashboard::PutString("S-StatusCodeName", Sstatus.GetName());  
       frc::SmartDashboard::PutString("S-StatusCodeDesc", Sstatus.GetDescription()); 
       
       // Display Status Code on SmartDashboard
    } 
    else {
       // Refresh config for LARGE angle configuration (was SMALL previously)
       configTurretMotor.MotionMagic.MotionMagicAcceleration = 11000_tr_per_s_sq; // acceleration  (ramp)
       configTurretMotor.MotionMagic.MotionMagicCruiseVelocity = 16000_tps;       // velocity  (once ramp up)
       configTurretMotor.MotionMagic.MotionMagicExpo_kV = (ctre::unit::volts_per_turn_per_second_t) 0.003; // Speed per unit of voltage (rotations/sec/V)
       configTurretMotor.MotionMagic.MotionMagicExpo_kA = (ctre::unit::volts_per_turn_per_second_squared_t)0.02; // Acceleration per unit of voltage (rotations/sec^2/V)
       ctre::phoenix::StatusCode Lstatus = turretMotor.GetConfigurator().Apply(configTurretMotor.MotionMagic);   
       frc::SmartDashboard::PutNumber("L-StatusCode", Lstatus);  
       frc::SmartDashboard::PutString("L-StatusCodeName", Lstatus.GetName());  
       frc::SmartDashboard::PutString("L-StatusCodeDesc", Lstatus.GetDescription());         
    }
    // Update LAST commanded angle information with CURRENT information
    m_lastCmdIsSmallAngle = isSmallAngle;
    // ***** REFRESH Motor Configuration (AS NEEDED ONLY)
    frc::SmartDashboard::PutNumber("TurretANGLE", angle);

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

    if (!m_isShotTableEnabled) {
        hoodAngle = m_hoodAngle;  // Use SmartDashboard variable
    } else {
        ShotSolutionEntry shotSolution = m_shotSolutionMap[shotTableIndex];      
        hoodAngle = shotSolution.hood_ANGLE;
    }
    frc::SmartDashboard::PutNumber("HoodANGLE", hoodAngle);

    return (hoodAngle);
}

double Turret::getHoodAngleDelta (frc::Pose2d robotPose) {
    
    // TBD - Determine Delta RPS from Robot Motion
    return (m_hoodAngleDelta);
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
   
    if (!m_isShotTableEnabled) {
        shooterRPS = m_shooterRPS;  // Use SmartDashboard variable
    } else {
        ShotSolutionEntry shotSolution = m_shotSolutionMap[shotTableIndex];      
        shooterRPS = shotSolution.shooter_RPS;
    }
    frc::SmartDashboard::PutNumber("ShooterRPS", shooterRPS);

    return (shooterRPS);
}

double Turret::getShooterRPSDelta (frc::Pose2d robotPose) {
    
    // TBD - Determine Delta RPS from Robot Motion
    return (m_shooterRPSDelta);
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
    ctre::phoenix6::controls::VelocityVoltage m_dualRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
    ctre::phoenix6::controls::VelocityVoltage m_singleRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

    feederDualMotor.SetControl(m_dualRequest.WithVelocity((units::angular_velocity::turns_per_second_t) feederRPS).WithFeedForward(0.5_V));
    feederSingleMotor.SetControl(m_singleRequest.WithVelocity((units::angular_velocity::turns_per_second_t) feederRPS).WithFeedForward(0.5_V));   
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
    // Set the motor speed
    ctre::phoenix6::controls::VelocityVoltage m_spindexerRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
    spindexerMotor.SetControl(m_spindexerRequest.WithVelocity((units::angular_velocity::turns_per_second_t) -(spindexerRPS * m_spindexerGearRatio)).WithFeedForward(0.5_V));
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
    // Convert RevolutionsPerMinute (RPM) to RevolutiuonsPerSecond (RPS)
    double rpm = m_intakeRPM;   // FIXED RPM
    double motorRPM = rpm * m_intakeGearRatio;
    // Convert RevolutionsPerMinute (RPM) to RevolutiuonsPerSecond (RPS)
    m_intakeRPS = motorRPM / 60.0;

    // Set the motor speeds (FORWARD and REVERSE)
    intakeMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) m_intakeRPS});
}

void Turret::zeroizeIntakePosition () {
    // ASSUMES THE INTAKE IS FULLY RETRACTED (NOT EXTENDED)
    ctre::phoenix6::controls::PositionDutyCycle initZeroRequest{(units::angle::turn_t) 0};
    deployMotor.SetControl (initZeroRequest);
    isIntakeActive = true;
}

void Turret::startIntake () {
    isIntakeActive = true;

      // deploy (fully extend)
    units::angle::turn_t intakeDeployPosition = (units::angle::turn_t) m_intakeDeployPosition;
    ctre::phoenix6::controls::MotionMagicDutyCycle extendRequest{intakeDeployPosition};
    deployMotor.SetControl(extendRequest);      
    
    setIntakeRPS ();
}

void Turret::stopIntake () {
    isIntakeActive = false;

    // Disable the Intake and Undeploy (fully retract)
    // Stop the Intake Motor
    intakeMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    intakeMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});

     // Undeploy (fully retract)
    units::angle::turn_t intakeZeroPosition = (units::angle::turn_t) 0;
    ctre::phoenix6::controls::MotionMagicDutyCycle retractRequest{intakeZeroPosition};
    deployMotor.SetControl(retractRequest);       
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
double Turret::computeRobotToTgtAngleInDegrees(frc::Pose2d turretPose, frc::Translation2d turretTarget )
{
    double pi_val = 4.0 * std::atan(1.0);       // Compute PI to many digits, atan (1 radian) = PI/4
    double RadiansToDegrees = 180.0 / pi_val;   // Radians to Degrees Conversion Factor
    //double DegreesToRadians = pi_val / 180.0; // Degrees to Radians Conversion Factor

    // Compute the Angle from the Robot X,Y Position to the Target X,Y Position
    double delta_X = (double) (turretTarget.X() - turretPose.X());
    double delta_Y = (double) (turretTarget.Y() - turretPose.Y());
    double robotToTgtAngleRadians = atan2(delta_Y, delta_X);                    // Radians
    double robotToTgtAngleDegrees = robotToTgtAngleRadians * RadiansToDegrees;  // Degrees

    return (robotToTgtAngleDegrees);
}

// make a robot velocity compensated vector for determining the turret angle, hood angle, and shooter velocity with compensation for the robot velocity.
Turret::ShotSetpoint Turret::getVelocityCompensatedShotSetpoint(double robotToTargetAngle, double shooterVelocityMPS, double hoodAngleDegrees) {
    double hoodAngleRadians = hoodAngleDegrees * M_PI / 180;
    double robotToTargetAngleRadians = robotToTargetAngle * M_PI / 180;
    auto shooterHorizontalVelocityMPS = shooterVelocityMPS * sin(hoodAngleRadians);
    auto shooterVerticalVelocityMPS = shooterVelocityMPS * cos(hoodAngleDegrees);
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
    double newTurretAngleDegrees = atan2(shooterVelocityVectorMPS[1],shooterVelocityVectorMPS[0]);
    double newShooterRPS = shooterVelocityVectorMPS.norm() / (2 * 0.0254 * M_PI);
    if (newHoodAngleDegrees < 10) {newHoodAngleDegrees = 10;}
    if (newHoodAngleDegrees > 40) {newHoodAngleDegrees = 40;}
    return ShotSetpoint{newTurretAngleDegrees, newShooterRPS, newHoodAngleDegrees};
}

// ****************************************

void Turret::enableTurretOperation ()     { startTurret();    }
void Turret::enableHoodOperation ()       { startHood();      }
void Turret::enableShooterOperation ()    { startShooter();   }
void Turret::enableFeederOperation ()     { startFeeder();    }
void Turret::enableSpindexerOperation ()  { startSpindexer(); }
void Turret::enableIntakeOperation ()     { startIntake();    }
void Turret::enableTopEndOperation ()     { startHood(); startShooter(); startFeeder(); startSpindexer();  }

void Turret::disableTurretOperation ()    { stopTurret();     }
void Turret::disableHoodOperation ()      { stopHood();       }
void Turret::disableShooterOperation ()   { stopShooter();    }
void Turret::disableFeederOperation ()    { stopFeeder();     }
void Turret::disableSpindexerOperation () { stopSpindexer();  }
void Turret::disableIntakeOperation ()    { stopIntake();  }
void Turret::disableTopEndOperation ()     { stopSpindexer(); stopFeeder(); stopShooter(); stopHood();  }



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

frc2::CommandPtr Turret::cmdOnIntake() {
   return RunOnce([this] { enableIntakeOperation(); }).WithName("Enable Intake Operation");
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

frc2::CommandPtr Turret::cmdOffIntake() {
   return RunOnce([this] { disableIntakeOperation(); }).WithName("Disable Spindexer Operation");
}


// Turret Destructor
Turret::~Turret() {}
