#include "subsystems/Turret.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
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
    configTurretMotor.MotionMagic.MotionMagicAcceleration = 10000_tr_per_s_sq; // acceleration  (ramp)
    configTurretMotor.MotionMagic.MotionMagicCruiseVelocity = 16000_tps;       // velocity  (once ramp up)
    configTurretMotor.MotionMagic.MotionMagicExpo_kV = (ctre::unit::volts_per_turn_per_second_t) 0.003; // Speed per unit of voltage (rotations/sec/V)
    configTurretMotor.MotionMagic.MotionMagicExpo_kA = (ctre::unit::volts_per_turn_per_second_squared_t)0.02; // Acceleration per unit of voltage (rotations/sec^2/V)
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

    // Clear all Delta Values
    m_turretTargetAngleDelta = 0;
    m_turretTargetDistanceDelta = 0;
    

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

        // Indicate the last command action, so it is NOT done again.
        m_lastCmdAction = cmdAction; 
    }

    m_turretAngle = frc::SmartDashboard::GetNumber("TurretANGLE", 0.0);
    frc::SmartDashboard::PutNumber("TurretANGLE", m_turretAngle);    
    frc::SmartDashboard::PutNumber("TurretTURNS", m_turretTurns);

    m_hoodAngle = frc::SmartDashboard::GetNumber("HoodANGLE", 0.0);
    frc::SmartDashboard::PutNumber("HoodANGLE", m_hoodAngle);
    frc::SmartDashboard::PutNumber("HoodTURNS", m_hoodTurns);
  
    m_shooterRPS = frc::SmartDashboard::GetNumber("ShooterRPS", 10.0);
    frc::SmartDashboard::PutNumber("shootRPS", m_shooterRPS);

    m_feederRPS = frc::SmartDashboard::GetNumber("FeederRPS", 3.4);
    frc::SmartDashboard::PutNumber("feedRPS", m_feederRPS);

    m_spindexerRPS = frc::SmartDashboard::GetNumber("SpindexerRPS", 3.4);
    frc::SmartDashboard::PutNumber("spinRPS", m_spindexerRPS);

    m_intakeRPM = frc::SmartDashboard::GetNumber("IntakeRPM", 10.0);
    frc::SmartDashboard::PutNumber("intakeRPM", m_intakeRPM);

    frc::SmartDashboard::PutNumber("shooterRPS", m_shooterRPS);
    frc::SmartDashboard::PutNumber("feederRPS", m_feederRPS);
    frc::SmartDashboard::PutNumber("spindexerRPS", m_spindexerRPS);
    frc::SmartDashboard::PutNumber("intakeRPS", m_intakeRPS);


    // ****************************************
    // Determine Turret Angle and Distance to the Target
    // (Since Turret Target has been Identified) 
    if (m_turretTargetSet)
    {
        // Get the current robot pose
        frc::Pose2d  m_pose = m_drivePtr->getPose();

        // Determine the Turret Pose (Relative to the Robot Pose)
        m_turretPose = frc::Pose2d{m_pose.Translation()+m_turretTranslation.RotateBy(m_pose.Rotation()), m_pose.Rotation() + frc::Rotation2d{180_deg}};

        // Compute Shooting solution (stationary) Turret Angle and Distance
        // The turret angle will point to the target, compensating for robot heading.
        m_turretTargetAngle    = computeTurretAngleInDegrees(m_turretPose,
                                                             m_turretTarget );
        m_turretTargetDistance = computeDistanceInMeters(m_turretPose.X().value(),
                                                         m_turretPose.Y().value(),
                                                         m_turretTarget.X().value(),
                                                         m_turretTarget.Y().value());

                                                     
        // **********
        // Adjust the Turret Target Angle and Distance for the motion of the Robot
        // Allow shooting while moving
        // Adjust Turret Target Angle (if desired,include robot motion)
        m_turretTargetAngle += m_turretTargetAngleDelta;
        // Adjust Turret Target Distance (if desired, include robot motion)
        m_turretTargetDistance += m_turretTargetDistanceDelta;    
        // **********

        // ************************************************
        // Adjust the Turret Motors for Proper Operation
        // SET THE TURRET (if enabled) for proper shooting direction to the target
        if (isTurretActive) {
            // *****
            // Point Turret to the Requested Angle
            // setTurretPosition(m_turretTargetAngle);
            // *****

            // TESTING - Temporarily, get angle from SmartDashboard variable
            setTurretPosition(m_turretAngle);
        }
        // SET THE HOOD (if enabled) for proper shooting angle to the target
        if (isHoodActive) {
            // *****
            // Determine hood angle from distance table lookup and Robot motion
            double hoodAngle = getHoodAngle(m_turretTargetDistance) + getHoodAngleDelta (m_pose);
            setHoodPosition (hoodAngle);
        }
        // SET THE SHOOTER (if enabled) for proper shooting speed to the target
        if (isShooterActive) {
            // *****
            // Get SHOOTER rps fom distance table lookup shoot RPS from distance table lookup
            double shooterRPS = getShooterRPS(m_turretTargetDistance) + getShooterRPSDelta (m_pose);
            setShooterRPS (shooterRPS);
        }
        // SET THE FEEDER (if enabled) for proper feeding to the shooter
        if (isFeederActive) { 
            setFeederRPS ();
        }
 
        // SET THE SPINDEXER (if enabled) for proper feeding into the shooter
        if (isSpindexerActive) {         
            setSpindexerRPS ();
        }

        // SET THE INTAKE (if enabled) for proper intake into the Spindexer
        if (isIntakeActive) {         
            setIntakeRPM ();
        }      
    }
    // ****************************************       

    // **************************************** 
    frc::SmartDashboard::PutNumber("TurretTargetX", m_turretTarget.X().value());  
    frc::SmartDashboard::PutNumber("TurretTargetY", m_turretTarget.Y().value());  
 
    frc::SmartDashboard::PutNumber("TurretX", m_turretPose.X().value());  
    frc::SmartDashboard::PutNumber("TurretY", m_turretPose.Y().value());    
    frc::SmartDashboard::PutNumber("TurretAngle", m_turretTargetAngle);  
    frc::SmartDashboard::PutNumber("TurretDistance", m_turretTargetDistance);  
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
    }
    // Ensure angle NOT LESS than minimum
    if (reqAngle < m_MinTurretAngle) {
        reqAngle = m_MinTurretAngle;
    }
 
    double mechRotations = reqAngle / 360.0;
    m_turretTurns = mechRotations * m_turretGearRatio;
    units::angle::turn_t turns = (units::angle::turn_t) m_turretTurns;

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
    isTurretActive = true;
}

void Turret::stopTurret () {
    isTurretActive = false;
}


double Turret::getHoodAngle (double distance) {
    double hoodAngle = 0;

    // Determine the hood angle from lookup table
    // Convert Distance to SHOT SOLUTION MAP table entry index
    int shotTableIndex = (int) ((distance / 3.0) * 10);           // 1/3 meter steps
 
    // TESTING
    shotTableIndex = 0;
    // TESTING
 
    if (shotTableIndex == 0) {
        hoodAngle = m_hoodAngle;
    } else {
        ShotSolutionEntry shotSolution = m_shotSolutionMap[shotTableIndex];      
        hoodAngle = shotSolution.hood_ANGLE;
    }

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
    isHoodActive = true;
}

void Turret::startHood () {
    isHoodActive = true;
}

void Turret::stopHood () {
    // Disable the Hood and Stow (fully retract to allow Trench passage)
    ctre::phoenix6::controls::PositionVoltage hoodRequest = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);   
    hoodMotor.SetControl(hoodRequest.WithPosition(0_tr));  // Hood does NOT fully retract immediatly at zeroized point 
    isHoodActive = false;
}



// *** SHOOTER ***
double Turret::getShooterRPS (double distance) {
   double shooterRPS = 0;

    // Determine the hood angle from lookup table
    // Convert Distance to SHOT SOLUTION MAP table entry index
    int shotTableIndex = (int) ((distance / 3.0) * 10);           // 1/3 meter steps

     // TESTING
    shotTableIndex = 0;
    // TESTING
   
    if (shotTableIndex == 0) {
        shooterRPS = m_shooterRPS;
    } else {
        ShotSolutionEntry shotSolution = m_shotSolutionMap[shotTableIndex];      
        shooterRPS = shotSolution.shooter_RPS;
    }

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
}

// *** FEEDER ***
void Turret::setFeederRPS () {
    // Set the motor speeds (DUAL and SINGLE)
    ctre::phoenix6::controls::VelocityVoltage m_dualRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
    ctre::phoenix6::controls::VelocityVoltage m_singleRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

    feederDualMotor.SetControl(m_dualRequest.WithVelocity((units::angular_velocity::turns_per_second_t) m_feederRPS).WithFeedForward(0.5_V));
    feederSingleMotor.SetControl(m_singleRequest.WithVelocity((units::angular_velocity::turns_per_second_t) m_feederRPS).WithFeedForward(0.5_V));   
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
}

// *** SPINDEXER ***
void Turret::setSpindexerRPS () {
    // Set the motor speed
    ctre::phoenix6::controls::VelocityVoltage m_spindexerRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
    spindexerMotor.SetControl(m_spindexerRequest.WithVelocity((units::angular_velocity::turns_per_second_t) -(m_spindexerRPS * m_spindexerGearRatio)).WithFeedForward(0.5_V));
}

void Turret::startSpindexer () {
    isSpindexerActive = true;
}

void Turret::stopSpindexer () {
    isSpindexerActive = false;


    spindexerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t) 0});
    spindexerMotor.SetControl(ctre::phoenix6::controls::NeutralOut{});
}

// *** INTAKE ***
void Turret::setIntakeRPM () {
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
    
    setIntakeRPM ();
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
double Turret::computeTurretAngleInDegrees(frc::Pose2d turretPose, frc::Translation2d turretTarget )
{
    double pi_val = 4.0 * std::atan(1.0);       // Compute PI to many digits, atan (1 radian) = PI/4
    double RadiansToDegrees = 180.0 / pi_val;   // Radians to Degrees Conversion Factor
    //double DegreesToRadians = pi_val / 180.0; // Degrees to Radians Conversion Factor
    double turretAngleDegrees = 0.0;
    double MAX_TURRET_ROTATION_ANGLE = 179.5;   // Maximum Turret Rotation Angle (Either Direction)
    //double TurretAngleRadians = 0.0;

    // Compute the Angle from the Robot X,Y Position to the Target X,Y Position
    double delta_X = (double) (turretTarget.X() - turretPose.X());
    double delta_Y = (double) (turretTarget.Y() - turretPose.Y());
    double robotToTgtAngleRadians = atan2(delta_Y, delta_X);                    // Radians
    double robotToTgtAngleDegrees = robotToTgtAngleRadians * RadiansToDegrees;  // Degrees

    // Turret Angle to Target -  based on Robot Heading
    turretAngleDegrees = robotToTgtAngleDegrees - (double) (turretPose.Rotation().Degrees());
 
    // Keep Turret Angle within physical limits
    double turretAngleChange = turretAngleDegrees - lastTurretAngle;
    am::limitDegrees(turretAngleChange);
    turretAngleDegrees = lastTurretAngle + turretAngleChange;
    while (turretAngleDegrees > MAX_TURRET_ROTATION_ANGLE) {
        turretAngleDegrees -= 360;
    }
    while (turretAngleDegrees < -MAX_TURRET_ROTATION_ANGLE) {
        turretAngleDegrees += 360;
    }


    lastTurretAngle = turretAngleDegrees;

    return (turretAngleDegrees);
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
