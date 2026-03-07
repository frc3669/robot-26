// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/GenericHID.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "subsystems/ScoringMech.h"
#include "subsystems/Swerve.h"
#include "subsystems/Climb.h"
#include "subsystems/Turret.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  void ConfigureChooser();
  void ConfigureDefaultCommands();
  void DisplaySchedulerDetails();
  void InitializeOdometry();
  
 // ScoringMech m_scoringMech{&m_XKeys};

 private:
  frc::GenericHID m_driverController{0};
  frc2::CommandGenericHID m_XKeys{1};
  frc2::CommandGenericHID m_cmdDriverController{0};
  
  // subsystems...
  Swerve m_drive{0};
  // Climb m_climber{};
  Turret m_turret{&m_drive, &m_XKeys};

  // autonomous routines
  std::optional<frc2::CommandPtr> m_centerAuto;

  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureBindings();
  void RegisterNamedCommands();
};
