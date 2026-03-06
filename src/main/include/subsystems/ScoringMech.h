// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <map>

class ScoringMech : public frc2::SubsystemBase {
 public:
  ScoringMech(frc2::CommandGenericHID *xkeys);


  /**
   * intake coral from the feeder station
   */
  frc2::CommandPtr intake();
  frc2::CommandPtr coralReset();
  frc2::CommandPtr intakeAlgae();
  frc2::CommandPtr intakeCoralAlgae();
  frc2::CommandPtr intakeL2_5();
  frc2::CommandPtr intakeL3_5();
  frc2::CommandPtr ejectAlgae();
  frc2::CommandPtr home();
  frc2::CommandPtr setHeightAndAnglesCmd(float height, float coralAngle, float algaeAngle);
  frc2::CommandPtr goL4();
  frc2::CommandPtr goL3();
  frc2::CommandPtr goL2();
  frc2::CommandPtr goL1();
  frc2::CommandPtr setCoralScoringLevel(const int & level);
  frc2::CommandPtr goToCoralScoringPosition();
  frc2::CommandPtr ejectCoral();
  frc2::CommandPtr goBarge();
  frc2::CommandPtr goProcessor();
  frc2::CommandPtr prepareForClimb();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  void setIntakeSpeed(float speed);
  void setAlgaeIntakeSpeed(float speed);
  void setHeight(float height);
  void setCoralAngle(float angle);
  void setAlgaeAngle(float angle);
  void setHeightAndAngles(float height, float coralAngle, float algaeAngle);
  void brakeIntake();
  // sets the intake speed to eject coral depending on the current position
  void setEjectCoralSpeed();
  void setEjectAlgaeSpeed();
  void brakeAlgaeIntake();
  void stopEverything();
  bool getHeightReached(float height);
  bool getCoralAngleReached(float angle);
  bool getAlgaeAngleReached(float angle);
  float getCoralAngle();
  float getAlgaeAngle();
  float getHeight();
  void configureMotors();

  frc::DigitalInput intakeSensor{0};
  frc::DigitalInput algaeIntakeSensor{2};
  ctre::phoenix6::hardware::TalonFX elevatorMotor{41, ctre::phoenix6::CANBus("rio")};
  ctre::phoenix6::hardware::TalonFXS scoringMotor{61, ctre::phoenix6::CANBus("rio")};
  ctre::phoenix6::hardware::TalonFXS algaeScoringMotor{62, ctre::phoenix6::CANBus("rio")};
  ctre::phoenix6::hardware::TalonFX coralAngleMotor{51, ctre::phoenix6::CANBus("rio")};
  ctre::phoenix6::hardware::TalonFX algaeAngleMotor{52, ctre::phoenix6::CANBus("rio")};
  ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC positionCtrl{0_tr};
  frc2::CommandGenericHID *xkeys;
  int m_coralScoringLevel = 1;
};
