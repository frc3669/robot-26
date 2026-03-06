#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/GenericHID.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include "subsystems/SwerveModule.h"
#include <frc/Timer.h>
//#include <frc/Matrix.h>
#include <units/time.h>

#include <frc/smartdashboard/Field2d.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include "Constants.h"
#include "util.h"
#include <LimelightHelpers.h>


class Swerve : public frc2::SubsystemBase 

{
  public:
    Swerve(int driverControllerPortNum);
    void Periodic() override;
    void SimulationPeriodic() override;
    frc2::CommandPtr defaultDrive();
    void brake();
    frc2::CommandPtr driveToPole(const bool & isLeft);
    frc2::CommandPtr driveToPoleIntermediate(const bool & isLeft);
    void InitializeOdometry();
    bool reefWithinRange();
    bool safeToMoveCoralManipulator();
    ~Swerve();

   
    // ****************************************
    // get the current pose of the robot
    frc::Pose2d getPose();

  private:
    frc::GenericHID m_driverController;
    ctre::phoenix6::hardware::Pigeon2 gyro{1, ctre::phoenix6::CANBus("CTREdevices")};
    ctre::phoenix6::StatusSignal<units::angle::degree_t> * m_gyroAngleSignal;
    std::vector<ctre::phoenix6::BaseStatusSignal*> m_statusSignals;
    frc::Timer autoTimer;
    frc::Timer positionReachedTimer;
    // robot swerve modules
    SwerveModule m_frontLeft  = SwerveModule(1), 
                 m_frontRight = SwerveModule(4),
                 m_backLeft   = SwerveModule(2),
                 m_backRight  = SwerveModule(3) ;
    // array of module pointers
    SwerveModule * m_moduleList[4] = { &m_frontLeft, 
                                       &m_frontRight,
                                       &m_backLeft,
                                       &m_backRight };

    frc::Translation2d m_frontLeftLocation{12_in, 12_in};
    frc::Translation2d m_frontRightLocation{12_in, -12_in};
    frc::Translation2d m_backLeftLocation{-12_in, 12_in};
    frc::Translation2d m_backRightLocation{-12_in, -12_in};
    
    /**
     * swerve kinematics object for calculating the module states
     * given the positions of the modules and the chassis speeds 
     * NOTE: THE Order is important....Changed to FL<FR< BL< BR*/
    frc::SwerveDriveKinematics<4> m_kinematics 
    {
      m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation
    };

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator 
    {
      m_kinematics,
      frc::Rotation2d{0.0_rad},                                // Initial Gyro Angle
      { m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
        m_backLeft.GetPosition(),  m_backRight.GetPosition() },
      frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}},     //Initial Field Position
      {0.1, 0.1, 0.1},                                         // State Std Devs (Trust Robot)
      {0.7, 0.7, 9999999}                                      // Vision Std Devs () - NO CAM
    };

    void UpdateVision(const std::string& name);

    std::shared_ptr<nt::NetworkTable> m_llFront;
    std::shared_ptr<nt::NetworkTable> m_llRear;

    // slew limiter object that limits the rate at which we approach the target chassis speeds
    Util::SlewLimiter m_slewLimiter;
    // current robot pose
    frc::Pose2d m_pose;
    // last known limelight pose
    frc::Pose2d m_lastLimelightPose;
    // target pose for autonomous positioning during teleop
    frc::Pose2d m_targetPose;

    

    // rotating buffer to store past odometry positions
    frc::Translation2d m_pastTranslations[100];
    // the number of translations stored in the buffer that are relavent
    int m_validPastTranslationCount = 0;
    // index of the most recent odometry translation stored in the buffer
    int m_currentTranslationIndex = 0;
    // field for displaying on SmartDashboard
    frc::Field2d field;
    // pathplanner robot config
    std::optional<pathplanner::RobotConfig> robotConfig;

    // drive to the currently set target pose
    void driveToTargetPose();
    // returs true if the currently set target pose has been reached
    bool targetPoseReached();
    // returns true when the target pose has been withing range for the given settling time
    bool targetPoseReachedFor(units::second_t settleTime);
    // sets the robot's current rotation to the new rotation specified
    void resetRotation(frc::Rotation2d newRotation);
    // sets the robot's current pose to the new pose specified
    void resetPose(frc::Pose2d newPose);
    // drives the swerve using the joystick
    void driveTeleop();
    // drives the robot with the given chassis speeds
    void driveRobotRelative(const frc::ChassisSpeeds & robotRelativeSpeeds);
    // get the current robot relative chassis speeds
    frc::ChassisSpeeds getSpeeds();
    // get the current pose of the robot
    //    frc::Pose2d getPose();  Made Public
    // get coral scoring target pose for the given pole side
    frc::Pose2d getCoralScoringTargetPose(bool isLeft);
    // get intermediate scoring pose at a safe distance from the target to prevent the elevator from colliding
    frc::Pose2d getIntermediateCoralScoringPose(bool isLeft);
    // command the swerve chassis drive to a pose on the field
    frc2::CommandPtr driveToPose(const frc::Pose2d & targetPose, const units::second_t & settleTime);
    // thread that runs in the background to calculate the robot field location
    void OdometryThread();
};