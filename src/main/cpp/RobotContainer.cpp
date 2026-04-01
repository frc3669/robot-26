// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/Swerve.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/button/Trigger.h>
//#include "commands/MultiSubsystem.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/events/EventTrigger.h>

using namespace pathplanner;
using namespace MainConst;

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureBindings();
  RegisterNamedCommands();
  ConfigureChooser();
}

void RobotContainer::ConfigureBindings() {
  // Keypad Controls
  //
  m_XKeys.Button(1).OnTrue(m_turret.cmdOnTopEnd());
  m_XKeys.Button(2).OnTrue(m_turret.cmdOffTopEnd());
  m_XKeys.Button(3).OnTrue(m_turret.cmdRaiseIntake());
  m_XKeys.Button(4).OnTrue(m_turret.cmdRetractIntake());
  m_XKeys.Button(5).OnTrue(m_turret.cmdDeployIntake());

  m_XKeys.Button(7).OnTrue(m_turret.cmdUseCameras());

  m_XKeys.Button(13).OnTrue(m_turret.cmdReverseIntake());
  m_XKeys.Button(14).OnTrue(m_turret.cmdRevTopEnd());
  m_XKeys.Button(15).OnTrue(m_turret.cmdManualOperation());
  m_XKeys.Button(16).OnTrue(m_turret.cmdIntakeON_OFF());


  // scoring mechanism button bindings
  //m_XKeys.Button(9).OnTrue(GeneralCmds::IntakeSafely(m_drive, m_scoringMech));
  //m_XKeys.Button(17).OnTrue(GeneralCmds::HomeSafely(m_drive, m_scoringMech));
  //m_XKeys.Button(6).OnTrue(GeneralCmds::HomeSafely(m_drive, m_scoringMech));
  //m_XKeys.Button(16).OnTrue(m_scoringMech.coralReset());
  //m_XKeys.Button(15).OnTrue(m_scoringMech.setCoralScoringLevel(2));
  //m_XKeys.Button(14).OnTrue(m_scoringMech.setCoralScoringLevel(3));
  //m_XKeys.Button(13).OnTrue(m_scoringMech.setCoralScoringLevel(4));
  //m_XKeys.Button(10).OnTrue(m_scoringMech.ejectCoral());
  //m_XKeys.Button(5).OnTrue(m_scoringMech.intakeAlgae());
  //m_XKeys.Button(3).OnTrue(m_scoringMech.intakeL3_5());  // 
  //m_XKeys.Button(4).OnTrue(m_scoringMech.intakeL2_5());
  //m_XKeys.Button(1).OnTrue(m_scoringMech.goBarge());
  //m_XKeys.Button(2).OnTrue(m_scoringMech.goProcessor());
  //m_XKeys.Button(8).OnTrue(m_scoringMech.ejectAlgae());
  // climber control button bindings
  //m_XKeys.Button(7).OnTrue(m_scoringMech.prepareForClimb());
  //m_XKeys.Button(18).WhileTrue(m_climber.extend());
  //m_XKeys.Button(19).WhileTrue(m_climber.retract());
  // autoscore button bindings
  //m_XKeys.Button(12).OnTrue(Score::ScoreCoral(m_drive, m_scoringMech, false));
  //m_XKeys.Button(11).OnTrue(Score::ScoreCoral(m_drive, m_scoringMech, true));
  //m_cmdDriverController.Button(3).OnTrue(m_scoringMech.intakeCoralAlgae());
}

void RobotContainer::RegisterNamedCommands() {
  NamedCommands::registerCommand("Intake DEPLOY",  m_turret.cmdDeployIntake());
  NamedCommands::registerCommand("Intake RAISE",   m_turret.cmdRaiseIntake());
  NamedCommands::registerCommand("Intake RETRACT", m_turret.cmdRetractIntake());
  NamedCommands::registerCommand("TopEnd ON",  m_turret.cmdOnTopEnd());
  NamedCommands::registerCommand("TopEnd OFF", m_turret.cmdOffTopEnd());

  //NamedCommands::registerCommand("go L4", m_scoringMech.goL4());
  //NamedCommands::registerCommand("score right pole", Score::ScoreCoralForAuto(m_drive, m_scoringMech, false));
  //NamedCommands::registerCommand("score left pole", Score::ScoreCoralForAuto(m_drive, m_scoringMech, true));
}

void RobotContainer::ConfigureChooser() {

  m_TrenchOutpost = PathPlannerAuto("Trench Outpost").ToPtr();
  m_TrenchDepot = PathPlannerAuto("Trench Depot").ToPtr();
  m_Depot  = PathPlannerAuto("Depot").ToPtr();
  if (m_TrenchOutpost.has_value() &&  m_TrenchDepot.has_value() && m_Depot.has_value()) {
    m_chooser.SetDefaultOption("Trench Outpost", m_TrenchOutpost.value().get());
    m_chooser.AddOption("Trench Depot", m_TrenchDepot.value().get());
    m_chooser.AddOption("Depot", m_Depot.value().get());
  } 
  else {
    clog << "failed to get Autonomous Paths\n";
  }
  frc::SmartDashboard::PutData(&m_chooser);

  m_turret.m_shooterTgtChooser.SetDefaultOption("BLUEHub", "BLUEHub");
  m_turret.m_shooterTgtChooser.AddOption("BLUEDepot", "BLUEDepot");
  m_turret.m_shooterTgtChooser.AddOption("BLUEOutpost", "BLUEOutpost");
  m_turret.m_shooterTgtChooser.AddOption("REDHub", "REDHub");
  m_turret.m_shooterTgtChooser.AddOption("REDDepot", "REDDepot");
  m_turret.m_shooterTgtChooser.AddOption("REDOutpost", "REDOutpost");
  frc::SmartDashboard::PutData(&m_turret.m_shooterTgtChooser);

  m_turret.m_cmdActionChooser.SetDefaultOption("NoAction", "NoAction");
  m_turret.m_cmdActionChooser.AddOption("IntakeRETRACT", "IntakeRETRACT"); 
  m_turret.m_cmdActionChooser.AddOption("IntakeRAISE",   "IntakeRAISE"); 
  m_turret.m_cmdActionChooser.AddOption("IntakeDEPLOY",  "IntakeDEPLOY");
  m_turret.m_cmdActionChooser.AddOption("IntakeENABLE",  "IntakeENABLE");
  m_turret.m_cmdActionChooser.AddOption("IntakeDISABLE", "IntakeDISABLE");
  m_turret.m_cmdActionChooser.AddOption("IntakeREV",     "IntakeREV");
  m_turret.m_cmdActionChooser.AddOption("SpindexerOFF",  "SpindexerOFF");
  m_turret.m_cmdActionChooser.AddOption("SpindexerON",   "SpindexerON");
  m_turret.m_cmdActionChooser.AddOption("FeederOFF",     "FeederOFF");
  m_turret.m_cmdActionChooser.AddOption("FeederON",      "FeederON");
  m_turret.m_cmdActionChooser.AddOption("ShooterOFF",    "ShooterOFF");
  m_turret.m_cmdActionChooser.AddOption("ShooterON",     "ShooterON");
  m_turret.m_cmdActionChooser.AddOption("HoodOFF",       "HoodOFF");
  m_turret.m_cmdActionChooser.AddOption("HoodON",        "HoodON");
  m_turret.m_cmdActionChooser.AddOption("TurretOFF",     "TurretOFF");
  m_turret.m_cmdActionChooser.AddOption("TurretON",      "TurretON");
  m_turret.m_cmdActionChooser.AddOption("TopEndOFF",     "TopEndOFF");
  m_turret.m_cmdActionChooser.AddOption("TopEndON",      "TopEndON");
  m_turret.m_cmdActionChooser.AddOption("TopEndREV",     "TopEndREV");
  m_turret.m_cmdActionChooser.AddOption("ShotTableOFF",  "ShotTableOFF");
  m_turret.m_cmdActionChooser.AddOption("ShotTableON",   "ShotTableON");
  m_turret.m_cmdActionChooser.AddOption("CompOFF",       "CompOFF");
  m_turret.m_cmdActionChooser.AddOption("CompON",        "CompON");
  frc::SmartDashboard::PutData(&m_turret.m_cmdActionChooser); 
}

void RobotContainer::ConfigureDefaultCommands() {
  m_drive.SetDefaultCommand(std::move(m_drive.defaultDrive()));
  //m_climber.SetDefaultCommand(std::move(m_climber.brake()));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
}



void RobotContainer::DisplaySchedulerDetails() {
  //frc::SmartDashboard::PutData("Command Scheduler Status", &frc2::CommandScheduler::GetInstance());
  //frc::SmartDashboard::PutData("Swerve Status", &m_drive);
  //frc::SmartDashboard::PutData("Scoring Mechanism Status", &m_scoringMech);
  //frc::SmartDashboard::PutData("Climb Status", &m_climber);

}


void RobotContainer::InitializeOdometry() {
  m_turret.setTurretTarget (m_turret.m_BLUE_TgtHub);
  string selectedShooterTarget = m_turret.m_shooterTgtChooser.GetSelected();
  if (selectedShooterTarget == "BLUEHub") {
    m_turret.setTurretTarget (m_turret.m_BLUE_TgtHub);
  }
  else if (selectedShooterTarget == "REDHub") {
    m_turret.setTurretTarget (m_turret.m_RED_TgtHub);
  }
  
  m_drive.InitializeOdometry();
  
}