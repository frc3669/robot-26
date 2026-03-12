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
    configTurretMotor.Slot0.kP = 0.2;
    configTurretMotor.Slot0.kI = 0.0;
    configTurretMotor.Slot0.kD = 0.1;
    turretMotor.GetConfigurator().Apply(configTurretMotor);
 
    // Hood Motor (Position)
    configHoodMotor.Slot0.kP = 0.2;
    configHoodMotor.Slot0.kI = 0.0;
    configHoodMotor.Slot0.kD = 0.1;
    hoodMotor.GetConfigurator().Apply(configHoodMotor);


    // Shooter Motor(s) (Speed)  SAME CONFIG IS USED FOR BOTH MOTORS
    configShooterForwardMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configShooterForwardMotor.Slot0.kP = -0.1;
    configShooterForwardMotor.Slot0.kI = 0;
    configShooterForwardMotor.Slot0.kD = 0;
    configShooterForwardMotor.Slot0.kV = -0.1;
    shooterForwardMotor.GetConfigurator().Apply(configShooterForwardMotor);
    configShooterReverseMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configShooterReverseMotor.Slot0.kP = -0.1;
    configShooterReverseMotor.Slot0.kI = 0;
    configShooterReverseMotor.Slot0.kD = 0;
    configShooterReverseMotor.Slot0.kV = -0.1;
    shooterReverseMotor.GetConfigurator().Apply(configShooterReverseMotor);
 
    // Feeder Motor(s) (Speed)  SAME CONFIG IS USED FOR BOTH MOTORS
    configFeederDualMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configFeederDualMotor.Slot0.kP = -0.1;
    configFeederDualMotor.Slot0.kI = 0;
    configFeederDualMotor.Slot0.kD = 0;
    configFeederDualMotor.Slot0.kV = -0.1;
    feederDualMotor.GetConfigurator().Apply(configFeederDualMotor);
    configFeederSingleMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configFeederSingleMotor.Slot0.kP = -0.1;
    configFeederSingleMotor.Slot0.kI = 0;
    configFeederSingleMotor.Slot0.kD = 0;
    configFeederSingleMotor.Slot0.kV = -0.1;
    feederSingleMotor.GetConfigurator().Apply(configFeederSingleMotor);
 
    // Spindexer Motor (Speed)
    configSpindexerMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    configSpindexerMotor.Slot0.kP = -0.1;
    configSpindexerMotor.Slot0.kI = 0;
    configSpindexerMotor.Slot0.kD = 0;
    configSpindexerMotor.Slot0.kV = -0.1;
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
    frc::SmartDashboard::PutNumber("FeederRPS", m_feederRPS);
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
        // Indicate the last command action, so it is NOT done again.
        m_lastCmdAction = cmdAction; 
    }

    
    m_shooterRPS = frc::SmartDashboard::GetNumber("ShooterRPS", 10.0);
    frc::SmartDashboard::PutNumber("shootRPS", m_shooterRPS);

    m_feederRPS = frc::SmartDashboard::GetNumber("FeederRPS", 3.4);
    frc::SmartDashboard::PutNumber("feedRPS", m_feederRPS);

    m_spindexerRPS = frc::SmartDashboard::GetNumber("SpindexerRPS", 3.4);
    frc::SmartDashboard::PutNumber("spinRPS", m_spindexerRPS);

    m_intakeRPM = frc::SmartDashboard::GetNumber("IntakeRPM", 10.0);
    frc::SmartDashboard::PutNumber("intakeRPM", m_intakeRPM);

    frc::SmartDashboard::PutNumber("turretRPS", m_turretRPS);
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
        // Adjust Turret Target Angle (if desired,include robot motion)
        m_turretTargetAngle += m_turretTargetAngleDelta;

        m_turretTargetDistance = computeDistanceInMeters(m_turretPose.X().value(),
                                                         m_turretPose.Y().value(),
                                                         m_turretTarget.X().value(),
                                                         m_turretTarget.Y().value());
        // Adjust Turret Target Angle (if desired, include robot motion)
        m_turretTargetDistance += m_turretTargetDistanceDelta;
                                                     


        // ************************************************
        // Adjust the Turret Motors for Proper Operation
        // SET THE TURRET (if enabled) for proper shooting direction to the target
        if (isTurretActive) {
            setTurretPosition(m_turretTargetAngle);
        }
        // SET THE HOOD (if enabled) for proper shooting angle to the target
        if (isHoodActive) {
            setHoodPosition (m_turretTargetDistance);
        }
        // SET THE SHOOTER (if enabled) for proper shooting speed to the target
        if (isShooterActive) {
            // Determine shoot RPS from distance table lookup
            m_shooterRPS = getShooterRPS (m_turretTargetDistance);
            // TBD
            // Temporarily, just set RPS from variable, set by SmartDashboard
            setShooterRPS (m_shooterRPS);
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
    double mechRotations = angle / 360.0;
    m_turretRPS = mechRotations * m_turretGearRatio;
    units::angle::turn_t turns = (units::angle::turn_t) m_turretRPS;
    ctre::phoenix6::controls::MotionMagicDutyCycle request{turns};
    turretMotor.SetControl(request);
 }
    
 void Turret::zeroizeTurretPosition () {
    // ASSUMES THE TURRET HAS BEEN MANUALLY ALIGNED, FACING 0 degrees FORWARD, SAME AS ROBOT FRONT
    ctre::phoenix6::controls::PositionDutyCycle initZeroRequest{(units::angle::turn_t) 0};
    turretMotor.SetControl (initZeroRequest);
    isTurretActive = true;
 }

void Turret::startTurret () {
    isTurretActive = true;
}

void Turret::stopTurret () {
    // Disable the Hood and Stow (fully retract to allow Trench passage)
    units::angle::turn_t turretZeroPosition = (units::angle::turn_t) 0;

    isTurretActive = false;
    ctre::phoenix6::controls::MotionMagicDutyCycle turretRequest{turretZeroPosition};
    turretMotor.SetControl(turretRequest);    
}

// *** HOOD ***
void Turret::setHoodPosition (double angle) {
    // Ensure angle less than maximum
    if (angle > m_MaxHoodAngle) {
        angle = m_MaxHoodAngle;
    }
    double mechRotations = angle / 360.0;
    double motorRotations = mechRotations * m_hoodGearRatio;
    units::angle::turn_t turns = (units::angle::turn_t) motorRotations;   
    ctre::phoenix6::controls::MotionMagicDutyCycle request{turns};
    hoodMotor.SetControl(request);
}
    
void Turret::zeroizeHoodPosition () {
    // ASSUMES THE HOOD IS FULLY LOWERED (NOT EXTENDED)
    ctre::phoenix6::controls::PositionDutyCycle initZeroRequest{(units::angle::turn_t) 0};
    hoodMotor.SetControl (initZeroRequest);
    isHoodActive = true;
}

void Turret::startHood () {
    isHoodActive = true;
}

void Turret::stopHood () {
    // Disable the Hood and Stow (fully retract to allow Trench passage)
    units::angle::turn_t hoodZeroPosition = (units::angle::turn_t) 0;

    isHoodActive = false;
    ctre::phoenix6::controls::MotionMagicDutyCycle hoodRequest{hoodZeroPosition};
    hoodMotor.SetControl(hoodRequest);    
}



// *** SHOOTER ***
double Turret::getShooterRPS (double distance) {
    // Look up RPS from Distance Table
    // TBD
    return (m_shooterRPS);
}

void Turret::setShooterRPS (double rps) {
    // Set the motor speeds (FORWARD and REVERSE)
    shooterForwardMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t)-rps});
    shooterReverseMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t)+rps});
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
    // Set the motor speeds (FORWARD and REVERSE)
    feederDualMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t)-m_feederRPS});
    feederSingleMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(units::angular_velocity::turns_per_second_t)-m_feederRPS});
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
 
    ctre::phoenix6::controls::VelocityVoltage velocityControlRequest {(units::angular_velocity::turns_per_second_t) m_spindexerRPS};
 
    // Set the motor speed
    spindexerMotor.SetControl(velocityControlRequest);
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

void Turret::disableTurretOperation ()    { stopTurret();     }
void Turret::disableHoodOperation ()      { stopHood();       }
void Turret::disableShooterOperation ()   { stopShooter();    }
void Turret::disableFeederOperation ()    { stopFeeder();     }
void Turret::disableSpindexerOperation () { stopSpindexer();  }
void Turret::disableIntakeOperation ()    { stopIntake();  }

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
