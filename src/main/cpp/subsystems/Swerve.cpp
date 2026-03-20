#include "subsystems/Swerve.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>
#include <thread>

using namespace SwerveConstants;
using namespace MainConst;
using namespace DriverControllerConstants;
using namespace ctre::phoenix6;
using namespace pathplanner;

Swerve::Swerve(int driverControllerPortNum) : 
        m_driverController(driverControllerPortNum)
{
    // add all the status signals to a list for syncronized updates
    m_gyroAngleSignal = new StatusSignal(gyro.GetYaw());
    m_statusSignals.push_back(m_gyroAngleSignal);
    for (auto & module : m_moduleList) {
        m_statusSignals.push_back(module->m_driveMotorTurns);
        m_statusSignals.push_back(module->m_driveMotorVelocity);
        m_statusSignals.push_back(module->m_encoderTurns);
    }
    // set all the swerve status signals to update really fast
    BaseStatusSignal::SetUpdateFrequencyForAll(200_Hz, m_statusSignals);
    // configure AutoBuilder
    robotConfig = RobotConfig::fromGUISettings();
    AutoBuilder::configure(
        [this]() {return getPose();},
        [this](const frc::Pose2d& pose) {resetPose(pose);},
        [this]() {return getSpeeds();},
        [this](const frc::ChassisSpeeds& robotRelativeSpeeds) {driveRobotRelative(robotRelativeSpeeds);},
        std::make_shared<PPHolonomicDriveController>(
            SwerveConstants::translationConstants,
            SwerveConstants::rotationConstants
        ),
        robotConfig.value(),
        []() {
            // boolean supplier that controls when the path will be flipped for the red alliance
            // this year, we aren't going to flip the paths, since the field is rotationally symmetric
            // auto alliance = frc::DriverStation::GetAlliance();
            // if (alliance) {
            //     return alliance.value() == frc::DriverStation::Alliance::kRed;
            // }
            return false;
        },
        this
    );
}


void Swerve::SimulationPeriodic() {}

void Swerve::Periodic() {
    // print some useful data to the dashboard
    frc::SmartDashboard::PutNumber("pose X", m_pose.X().value());
    frc::SmartDashboard::PutNumber("pose Y", m_pose.Y().value());
    frc::SmartDashboard::PutNumber("pose A", m_pose.Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber("target pose X", m_targetPose.X().value());
    frc::SmartDashboard::PutNumber("target pose Y", m_targetPose.Y().value());
    frc::SmartDashboard::PutNumber("target pose A", m_targetPose.Rotation().Degrees().value());
    // Show the gyro Yaw angle
    frc::SmartDashboard::PutNumber("gyro A= ", (double) gyro.GetYaw().GetValue().value());
}

void Swerve::driveTeleop() {
    // invert the controls or not depending on which side of the field 
    int invert = 1;
    // we are currently using two different coordinate systems for each side, so this is not necessary
    // if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    //     invert = -1;
    // }
    //if (m_driverController.GetRawButton(4)) {
    //    resetRotation(0_deg);
    //}
    Eigen::Vector2d velocity(-m_driverController.GetRawAxis(1) * invert, -m_driverController.GetRawAxis(0) * invert);
    double angularVelocity = -m_driverController.GetRawAxis(4);
    /**
     * apply deadbands in such a way that it is still
     * possible to drive the robot at arbitrarily slow speeds
     **/
    if (velocity.norm() > dB) {
        velocity *= (1.0 - dB/velocity.norm())/(1.0 - dB);
    } else { velocity = Eigen::Vector2d(0, 0); }
    if (abs(angularVelocity) > dB) {
        angularVelocity *= (1.0 - dB/abs(angularVelocity))/(1.0 - dB);
    } else { angularVelocity = 0; }
    // scale the target velocity so it reaches the max
    velocity *= SwerveConstants::max_m_per_sec.value();
    angularVelocity *= SwerveConstants::max_rad_per_sec.value();
    // set the field oriented chassis speeds target
    frc::ChassisSpeeds rawControllerFieldOrientedSpeeds = frc::ChassisSpeeds{units::velocity::meters_per_second_t{velocity[0]},
                                                            units::velocity::meters_per_second_t{velocity[1]},
                                                            units::angular_velocity::radians_per_second_t{angularVelocity}};
    // get the robot oriented version of the chassis speeds target
    frc::ChassisSpeeds robotOrientedSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(rawControllerFieldOrientedSpeeds, m_pose.Rotation());
    // get the module states (the states contain module speed and wheel angle values for driving the modules)
    auto states = m_kinematics.ToSwerveModuleStates(robotOrientedSpeeds);
    // rescale the robotOriented target speeds to ensure that no module exceeds its max velocity
    double desaturationValue = Util::desaturateChassisSpeeds(robotOrientedSpeeds, states);
    // rescale the fieldOriented target speeds by the same amount we rescaled the robot oriented speeds so we can run it throught the slew limiting algorithm
    frc::ChassisSpeeds fieldRelativeSpeeds = rawControllerFieldOrientedSpeeds * desaturationValue;
    // run the slew limiter on the field relative speeds
    m_slewLimiter.Run(fieldRelativeSpeeds, SwerveConstants::time_to_full_speed, 0.02_s);
    // calculate the final swerve module states 
    states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(m_slewLimiter.GetSpeeds(), m_pose.Rotation()));
    // drive all the swerve modules
    for (int i = 0; i < 4; i++) {
        m_moduleList[i]->setDesiredState(states[i]);
    }
}

frc2::CommandPtr Swerve::defaultDrive() {
    return Run([this] { driveTeleop(); }).WithName("Driving Teleoperated");
}

void Swerve::driveToTargetPose() {
    auto translationError = m_targetPose.Translation() - m_pose.Translation();
    auto rotationError = m_targetPose.Rotation() - m_pose.Rotation();
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            translationError.X()*position_P/1_s,
            translationError.Y()*position_P/1_s,
            rotationError.Radians()*heading_P/1_s,
            m_pose.Rotation());
    auto moduleStates = m_kinematics.ToSwerveModuleStates(speeds);
    m_kinematics.DesaturateWheelSpeeds(&moduleStates, max_limelight_m_per_sec);
    for (int i = 0; i < 4; i++) {
        m_moduleList[i]->setDesiredState(moduleStates[i]);
    }
}

bool Swerve::targetPoseReached() {
    return m_targetPose.Translation().Distance(m_pose.Translation()) < 0.5_in;
}

bool Swerve::targetPoseReachedFor(units::second_t settleTime) {
    if (!targetPoseReached()) {
        positionReachedTimer.Restart();
    }
    return positionReachedTimer.HasElapsed(settleTime);
}

void Swerve::driveRobotRelative(const frc::ChassisSpeeds & robotRelativeSpeeds) {
    auto states = m_kinematics.ToSwerveModuleStates(robotRelativeSpeeds);
    m_kinematics.DesaturateWheelSpeeds(&states, max_m_per_sec);
    for (int i = 0; i < 4; i++) {
        m_moduleList[i]->setDesiredState(states[i]);
    }
}

void Swerve::brake() {
    for (auto &module : m_moduleList) {
        module->brake();
    }
}

frc2::CommandPtr Swerve::driveToPole(const bool & isLeft) {
    return frc2::FunctionalCommand(
        [this, isLeft] { m_targetPose = getCoralScoringTargetPose(isLeft); },
        [this] { driveToTargetPose(); },
        [this] (bool x) { driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}); },
        [this] { return targetPoseReachedFor(0.5_s); },
        {this}
    ).ToPtr().WithName("driving to given reef pole");
}

frc::Pose2d Swerve::getPose() {
    return m_pose;
}

frc::ChassisSpeeds Swerve::getSpeeds() {
    return m_kinematics.ToChassisSpeeds(m_moduleList[0]->GetState(), m_moduleList[1]->GetState(),
                                        m_moduleList[2]->GetState(), m_moduleList[3]->GetState());
}

frc2::CommandPtr Swerve::driveToPoleIntermediate(const bool & isLeft) {
    return frc2::FunctionalCommand(
        [this, isLeft] { m_targetPose = getIntermediateCoralScoringPose(isLeft); },
        [this] { driveToTargetPose(); },
        [this] (bool x) { driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}); },
        [this] { return targetPoseReached(); },
        {this}
    ).ToPtr().WithName("driving to intermediate coral scoring pose");
}

frc::Pose2d Swerve::getCoralScoringTargetPose(bool isLeft) {
    frc::Translation2d translationFromBlueReef = m_pose.Translation() - blueReefTranslation;
    frc::Translation2d translationFromRedReef = m_pose.Translation() - redReefTranslation;
    frc::Pose2d targetPose;
    if (translationFromBlueReef.Norm() < 2.5_m) {
        auto reefSideAngle = frc::Rotation2d{60_deg * int((translationFromBlueReef.Angle().Degrees().value() + 390)/60)};
        auto targetTranslation = blueReefTranslation + frc::Translation2d{units::meter_t{reefSideAngle.Cos()}, units::meter_t{reefSideAngle.Sin()}} * reefToRobotDistance;
        auto targetRotation = frc::Rotation2d{reefSideAngle} + frc::Rotation2d{180_deg};
        targetPose = frc::Pose2d{targetTranslation, targetRotation};
    }
    if (translationFromRedReef.Norm() < 2.5_m) {
        auto reefSideAngle = frc::Rotation2d{60_deg * int((translationFromRedReef.Angle().Degrees().value() + 390)/60)};
        auto targetTranslation = redReefTranslation + frc::Translation2d{units::meter_t{reefSideAngle.Cos()}, units::meter_t{reefSideAngle.Sin()}} * reefToRobotDistance;
        auto targetRotation = frc::Rotation2d{reefSideAngle} + frc::Rotation2d{180_deg};
        targetPose = frc::Pose2d{targetTranslation, targetRotation};
    }
    if (isLeft) {
        return {targetPose.Translation() + frc::Translation2d{-6.5_in*targetPose.Rotation().Sin(), 6.5_in*targetPose.Rotation().Cos()}, targetPose.Rotation()};
    } else {
        return {targetPose.Translation() + frc::Translation2d{6.5_in*targetPose.Rotation().Sin(), -6.5_in*targetPose.Rotation().Cos()}, targetPose.Rotation()};
    }
}

frc::Pose2d Swerve::getIntermediateCoralScoringPose(bool isLeft) {
    auto targetPose = getCoralScoringTargetPose(isLeft);
    return {targetPose.Translation()
        + frc::Translation2d{units::meter_t{-targetPose.Rotation().Cos()},
                             units::meter_t{-targetPose.Rotation().Sin()}} * scoringOffsetMeters,
                             targetPose.Rotation()};
}

bool Swerve::reefWithinRange() {
    frc::Translation2d translationFromBlueReef = m_pose.Translation() - blueReefTranslation;
    frc::Translation2d translationFromRedReef = m_pose.Translation() - redReefTranslation;
    return translationFromBlueReef.Norm() < 2.5_m || translationFromRedReef.Norm() < 2.5_m;
}

bool Swerve::safeToMoveCoralManipulator() {
    frc::Translation2d translationFromBlueReef = m_pose.Translation() - blueReefTranslation;
    frc::Translation2d translationFromRedReef = m_pose.Translation() - redReefTranslation;
    return (translationFromBlueReef.Norm() > units::meter_t{safeReefDistanceMeters})
        && (translationFromRedReef.Norm() > units::meter_t{safeReefDistanceMeters});
}

void Swerve::resetRotation(frc::Rotation2d newRotation) {
    gyro.SetYaw(newRotation.Degrees());
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
        LimelightHelpers::SetRobotOrientation("limelight-left", newRotation.Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetRobotOrientation("limelight-right", newRotation.Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetRobotOrientation("limelight-back", newRotation.Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
    } else {
        LimelightHelpers::SetRobotOrientation("limelight-left", newRotation.Degrees().value()+180, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetRobotOrientation("limelight-right", newRotation.Degrees().value()+180, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetRobotOrientation("limelight-back", newRotation.Degrees().value()+180, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    m_pose = {m_pose.Translation(), newRotation};
}

void Swerve::resetPose(frc::Pose2d newPose) {
    gyro.SetYaw(newPose.Rotation().Degrees());
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
         LimelightHelpers::SetRobotOrientation("limelight-left", newPose.Rotation().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetRobotOrientation("limelight-right", newPose.Rotation().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetRobotOrientation("limelight-back", newPose.Rotation().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
    } else {
        LimelightHelpers::SetRobotOrientation("limelight-left", newPose.Rotation().Degrees().value()+180, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetRobotOrientation("limelight-right", newPose.Rotation().Degrees().value()+180, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetRobotOrientation("limelight-back", newPose.Rotation().Degrees().value()+180, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    m_pose = newPose;
}


void Swerve::UpdateVision(const std::string& name) {

    // Get the limelight pose, based upon team alliance (Red or Blue)
    LimelightHelpers::PoseEstimate llPose;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
        LimelightHelpers::SetRobotOrientation(name, m_gyroAngleSignal->GetValue().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
        llPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(name); 
    } else {
        LimelightHelpers::SetRobotOrientation(name, m_gyroAngleSignal->GetValue().value() + 180, 0.0, 0.0, 0.0, 0.0, 0.0);
        llPose = LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2(name); 
    }

    // Check if the measurement is valid (e.g. if you see an April Tag)
    if (llPose.tagCount > 0 && llPose.timestampSeconds != (units::time::second_t(0)) && LimelightHelpers::getTV(name)) {
        // Convert Pose3d to Pose2d if using a 2D estimator (common for FRC swerve)
        frc::Pose2d visionPose2d = llPose.pose;
        frc::SmartDashboard::PutNumber(name+" X= ", visionPose2d.X().value());
        frc::SmartDashboard::PutNumber(name+" Y= ", visionPose2d.Y().value());
        frc::SmartDashboard::PutNumber(name+" A= ", visionPose2d.Rotation().Degrees().value());

        // Get the correct timestamp. The helper function provides an FPGA-timestamp-aligned value.
        units::second_t imageCaptureTime = units::second_t{llPose.timestampSeconds};

        // Add the vision measurement to the pose estimator
        // Make sure the vision measurement timestamp aligns with your robot's internal timebase (FPGA time is typical)
        // NOTE: Only use the Robot gyro - do not use the camera vision angle.
        m_poseEstimator.AddVisionMeasurement(visionPose2d, imageCaptureTime);
    }
}

void Swerve::OdometryThread() {
    while (true) {
        BaseStatusSignal::WaitForAll(10_ms, m_statusSignals);

        // Update the Pose Estimation from the Robot Odometry
        m_poseEstimator.Update ( m_gyroAngleSignal->GetValue(),
                                 { m_frontLeft.GetPosition(), 
                                   m_frontRight.GetPosition(),
                                   m_backLeft.GetPosition(), 
                                   m_backRight.GetPosition()} );

        // Adjust for each Limelight Camera
        // LEFT
        UpdateVision("limelight-left");   
        // RIGHT
        UpdateVision("limelight-right");       
        // BACK
        UpdateVision("limelight-back");

        // Get the current robot pose
        m_pose = m_poseEstimator.GetEstimatedPosition();       
    }
}

void Swerve::InitializeOdometry() {
    for (auto & module : m_moduleList) {
        module->InitializeOdometry();
    }
    LimelightHelpers::SetIMUMode("limelight-left", 0);
    LimelightHelpers::SetIMUMode("limelight-right", 0);
    LimelightHelpers::SetIMUMode("limelight-back", 0);
    std::thread odometryThread(&Swerve::OdometryThread, this);
    odometryThread.detach();
}

Swerve::~Swerve() {
    delete m_gyroAngleSignal;
}