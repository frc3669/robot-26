#pragma once
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/current.h>
#include <frc/geometry/Translation2d.h>
#include <pathplanner/lib/config/PIDConstants.h>

// constants for the robot as a whole
namespace MainConst {
    inline constexpr double code_cycle_time = 0.02;
    inline constexpr frc::Translation2d blueReefTranslation{176.75_in, 158.5_in};
    inline constexpr frc::Translation2d redReefTranslation{514.13_in, 158.5_in};
    inline constexpr double reefToRobotDistance = 1.298;
    inline constexpr double scoringOffsetMeters = 0.14;
    inline constexpr double safeReefDistanceMeters = reefToRobotDistance + scoringOffsetMeters;
    inline constexpr units::meter_t kFieldLength = 16.54175_m;
    inline constexpr units::meter_t kFieldWidth  = 8.0137_m;
}

namespace DriverControllerConstants {
    inline constexpr double dB = 0.1;
}

// constants for the swerve
namespace SwerveConstants {
    inline constexpr units::time::second_t time_to_full_speed = 0.5625_s;
    inline constexpr units::current::ampere_t max_torque_current = 100_A;
    inline constexpr double motor_turns_per_wheel_turn = 6.03;
    inline constexpr double motor_turns_per_steering_turn = 287/11;
    inline constexpr double wheel_diameter_m = 0.1016;    // Trent had this value 0.10081;
    inline constexpr double motor_turns_per_m = motor_turns_per_wheel_turn / (wheel_diameter_m*M_PI);
    inline constexpr units::velocity::meters_per_second_t max_m_per_sec = 4.5_mps;
    inline constexpr units::velocity::meters_per_second_t max_limelight_m_per_sec = 1_mps;
    inline constexpr units::angular_velocity::radians_per_second_t max_rad_per_sec = 20.88_rad_per_s;
    // TODO:: check max_rad_per_sec

    inline constexpr double position_P = 5.0;
    inline constexpr double heading_P = 13.92;
    inline constexpr pathplanner::PIDConstants translationConstants(5.0, 0.0, 0.0);
    inline constexpr pathplanner::PIDConstants rotationConstants(5.0, 0.0, 0.0);
}

namespace ScoreMechConst {
    inline constexpr double elevator_in_to_rotations = 5.0/9*2.54;
    inline constexpr double angle_gear_ratio = 30;
    inline constexpr double algae_angle_gear_ratio = 54;
}