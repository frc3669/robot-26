// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ScoringMech.h"
#include "util.h"
#include "Constants.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;
using namespace ScoreMechConst;

ScoringMech::ScoringMech(frc2::CommandGenericHID *xkeys) {
  this->xkeys = xkeys;
  configureMotors();
  elevatorMotor.SetPosition(0_tr);
  coralAngleMotor.SetPosition(0_tr);
  algaeAngleMotor.SetPosition(0_tr);
}

void ScoringMech::setIntakeSpeed(float speed) {
  scoringMotor.Set(speed);
}

void ScoringMech::setAlgaeIntakeSpeed(float speed) {
  algaeScoringMotor.Set(speed);
}

void ScoringMech::setHeight(float height) {
  float rotations = -height*elevator_in_to_rotations;
  elevatorMotor.SetControl(positionCtrl.WithPosition(rotations*1.0_tr));
}

void ScoringMech::setCoralAngle(float angle) {
  float rotations = -angle/360.5*angle_gear_ratio;
  coralAngleMotor.SetControl(positionCtrl.WithPosition(rotations*1.0_tr));
}

void ScoringMech::setAlgaeAngle(float angle) {
  float rotations = angle/360.5*algae_angle_gear_ratio;
  algaeAngleMotor.SetControl(positionCtrl.WithPosition(rotations * 1.0_tr));
}

void ScoringMech::setHeightAndAngles(float height, float coralAngle, float algaeAngle) {
  setHeight(height);
  setCoralAngle(coralAngle);
  setAlgaeAngle(algaeAngle);
}

void ScoringMech::brakeIntake() {
  scoringMotor.SetControl(controls::NeutralOut());
}

void ScoringMech::setEjectCoralSpeed() {
  if (getCoralAngle() > 60) {
    scoringMotor.SetControl(controls::DutyCycleOut(0.25));
  } else if (getHeightReached(5)) {
    scoringMotor.SetControl(controls::DutyCycleOut(-0.70));
  } else {
    scoringMotor.SetControl(controls::DutyCycleOut(-0.25));
  }
}

void ScoringMech::setEjectAlgaeSpeed() {
  if (getHeight() > 20) {
    algaeScoringMotor.SetControl(controls::DutyCycleOut(-1));
  } else {
    algaeScoringMotor.SetControl(controls::DutyCycleOut(-0.4));
  }
}

void ScoringMech::brakeAlgaeIntake() {
  algaeScoringMotor.SetControl(controls::NeutralOut());
}

void ScoringMech::stopEverything() {
  brakeIntake();
  brakeAlgaeIntake();
}

frc2::CommandPtr ScoringMech::intake() {
  return frc2::cmd::Sequence(
    setHeightAndAnglesCmd(12, 32, 0),
    RunOnce([this] { setIntakeSpeed(0.25); }),
    frc2::cmd::WaitUntil([this] { return !intakeSensor.Get(); }),
    RunOnce([this] { setIntakeSpeed(-0.1); }),
    frc2::cmd::WaitUntil([this] { return intakeSensor.Get(); }),
    RunOnce([this] { setIntakeSpeed(0.1); }),
    frc2::cmd::WaitUntil([this] { return !intakeSensor.Get(); }),
    RunOnce([this] { brakeIntake(); })
  ).WithName("Intaking Coral");
}

frc2::CommandPtr ScoringMech::coralReset() {
  return frc2::cmd::Sequence(
    RunOnce([this] { setIntakeSpeed(-0.15); }),
    frc2::cmd::WaitUntil([this] { return intakeSensor.Get(); }),
    RunOnce([this] { setIntakeSpeed(0.1); }),
    frc2::cmd::WaitUntil([this] { return !intakeSensor.Get(); }),
    RunOnce([this] { brakeIntake(); })
  ).WithName("Resetting Coral");
}

frc2::CommandPtr ScoringMech::intakeAlgae() {
  return frc2::cmd::Sequence(
    setHeightAndAnglesCmd(0, 0, 170),
    RunOnce([this] { setAlgaeIntakeSpeed(0.5); }),
    frc2::cmd::WaitUntil([this] { return !algaeIntakeSensor.Get(); }),
    RunOnce([this] { brakeAlgaeIntake(); })
  ).WithName("Intaking Algae");
}

frc2::CommandPtr ScoringMech::intakeCoralAlgae() {
  return frc2::cmd::Sequence(
    setHeightAndAnglesCmd(0, 0, 120),
    RunOnce([this] { setAlgaeIntakeSpeed(0.5); }),
    frc2::cmd::WaitUntil([this] { return !algaeIntakeSensor.Get(); }),
    RunOnce([this] { brakeAlgaeIntake(); })
  ).WithName("Intaking Algae");
}

frc2::CommandPtr ScoringMech::intakeL2_5() {
  return frc2::cmd::Sequence(
    setHeightAndAnglesCmd(12, 0, 110),
    RunOnce([this] { setAlgaeIntakeSpeed(0.5); }),
    frc2::cmd::WaitUntil([this] { return !algaeIntakeSensor.Get(); }),
    RunOnce([this] { brakeAlgaeIntake(); })
  ).WithName("Intaking from level 2.5");
}

frc2::CommandPtr ScoringMech::intakeL3_5() {
  return frc2::cmd::Sequence(
    setHeightAndAnglesCmd(30, 0, 110),
    RunOnce([this] { setAlgaeIntakeSpeed(0.5); }),
    frc2::cmd::WaitUntil([this] { return !algaeIntakeSensor.Get(); }),
    RunOnce([this] { brakeAlgaeIntake(); })
  ).WithName("Intaking from level 3.5");
}

frc2::CommandPtr ScoringMech::home() {
  return frc2::cmd::Sequence(
    RunOnce([this] { stopEverything(); }),
    setHeightAndAnglesCmd(0,0,0),
    RunOnce([this] { elevatorMotor.SetControl(controls::NeutralOut()); })
  ).WithName("Homing");
}

frc2::CommandPtr ScoringMech::goL4() {
  return setHeightAndAnglesCmd(44.5, 95.65, 0).WithName("Going to Level 4");
}

frc2::CommandPtr ScoringMech::goL3() {
  return setHeightAndAnglesCmd(6, 154, 0).WithName("Going to Level 3");
}

frc2::CommandPtr ScoringMech::goL2() {
  return setHeightAndAnglesCmd(5, 3.65, 0).WithName("Going to Level 2");
}

frc2::CommandPtr ScoringMech::goToCoralScoringPosition() {
  return frc2::cmd::Select<int, frc2::CommandPtr>(
    [this] { return m_coralScoringLevel; },
    std::pair<int,frc2::CommandPtr>{1, home()},
    std::pair<int,frc2::CommandPtr>{2, goL2()},
    std::pair<int,frc2::CommandPtr>{3, goL3()},
    std::pair<int,frc2::CommandPtr>{4, goL4()}
    ).WithName("going to selected coral scoring position");
}

frc2::CommandPtr ScoringMech::setCoralScoringLevel(const int & level) {
  return RunOnce([this, level] { if (1 <= level && level <= 4) m_coralScoringLevel = level; })
    .WithName("setting given coral scoring level");
}

frc2::CommandPtr ScoringMech::ejectCoral() {
  return frc2::cmd::Sequence(
    RunOnce([this] { setEjectCoralSpeed(); }),
    frc2::cmd::Wait(0.25_s),
    RunOnce([this] { brakeIntake(); })
  ).WithName("Ejecting Coral");
}

frc2::CommandPtr ScoringMech::goBarge() {
  return frc2::cmd::Sequence(
    setHeightAndAnglesCmd(49, 0, 50)
  ).WithName("Going to Barge Pose");
}

frc2::CommandPtr ScoringMech::ejectAlgae() {
  return frc2::cmd::Sequence(
    RunOnce([this] { setEjectAlgaeSpeed(); }),
    frc2::cmd::Wait(0.75_s),
    RunOnce([this] { brakeAlgaeIntake(); })
  ).WithName("Ejecting Algae");
}

frc2::CommandPtr ScoringMech::goProcessor() {
  return frc2::cmd::Sequence(
    setHeightAndAnglesCmd(0, 0, 125)
  ).WithName("Going to Processor Pose");
}

frc2::CommandPtr ScoringMech::prepareForClimb() {
  return setHeightAndAnglesCmd(0,0,170).WithName("Preparing for Climb");
}

frc2::CommandPtr ScoringMech::setHeightAndAnglesCmd(float height, float coralAngle, float algaeAngle) {
  return frc2::cmd::Sequence(
    frc2::InstantCommand([this, height, coralAngle, algaeAngle] { setHeightAndAngles(height, coralAngle, algaeAngle); }, {this}).ToPtr(),
    frc2::cmd::WaitUntil([this, height, coralAngle, algaeAngle] { return (getHeightReached(height) && getCoralAngleReached(coralAngle) && getAlgaeAngleReached(algaeAngle)); })
  ).WithName("Setting Height and Angles");
}

float ScoringMech::getHeight() {
  return -elevatorMotor.GetPosition().GetValueAsDouble()/elevator_in_to_rotations;
}

bool ScoringMech::getHeightReached(float height) {
  return std::abs(getHeight() - height) < 1;
}

float ScoringMech::getCoralAngle() {
  return -coralAngleMotor.GetPosition().GetValueAsDouble()*360/angle_gear_ratio;
}

bool ScoringMech::getCoralAngleReached(float angle) {
  return std::abs(getCoralAngle() - angle) < 5;
}

float ScoringMech::getAlgaeAngle() {
  return algaeAngleMotor.GetPosition().GetValueAsDouble()*360/algae_angle_gear_ratio;
}

bool ScoringMech::getAlgaeAngleReached(float angle) {
  return std::abs(getAlgaeAngle() - angle) < 5;
}

void ScoringMech::Periodic() {
}

void ScoringMech::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void ScoringMech::configureMotors() {
  // coral angle motor configs
  configs::TalonFXConfiguration coralAngleCfg{};
  coralAngleCfg.Slot0.kP = 40;
  coralAngleCfg.Slot0.kS = 4;
  coralAngleCfg.TorqueCurrent.PeakForwardTorqueCurrent = 50_A;
  coralAngleCfg.TorqueCurrent.PeakReverseTorqueCurrent = -50_A;
  coralAngleCfg.MotionMagic.MotionMagicAcceleration = 80_tr_per_s_sq;
  coralAngleCfg.MotionMagic.MotionMagicCruiseVelocity = 80_tps;
  coralAngleCfg.MotionMagic.MotionMagicJerk = 300_tr_per_s_cu;
  // algae angle motor configs
  configs::TalonFXConfiguration algaeAngleCfg{};
  algaeAngleCfg.Slot0.kP = 30;
  algaeAngleCfg.Slot0.kS = 4;
  algaeAngleCfg.TorqueCurrent.PeakForwardTorqueCurrent = 30_A;
  algaeAngleCfg.TorqueCurrent.PeakReverseTorqueCurrent = -30_A;
  algaeAngleCfg.MotionMagic.MotionMagicAcceleration = 20_tr_per_s_sq;
  algaeAngleCfg.MotionMagic.MotionMagicCruiseVelocity = 80_tps;
  algaeAngleCfg.MotionMagic.MotionMagicJerk = 200_tr_per_s_cu;
  // elevator motor configs
  configs::TalonFXConfiguration elevatorCfg{};
  elevatorCfg.Slot0.kP = 40;
  elevatorCfg.Slot0.kS = 0;
  elevatorCfg.Slot0.kG = -18;
  elevatorCfg.TorqueCurrent.PeakForwardTorqueCurrent = 150_A;
  elevatorCfg.TorqueCurrent.PeakReverseTorqueCurrent = -200_A;
  elevatorCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
  elevatorCfg.CurrentLimits.SupplyCurrentLimit = 60_A;
  elevatorCfg.CurrentLimits.SupplyCurrentLowerTime = 0_s;
  elevatorCfg.MotionMagic.MotionMagicAcceleration = 150_tr_per_s_sq;
  elevatorCfg.MotionMagic.MotionMagicCruiseVelocity = 90_tps;
  elevatorCfg.MotionMagic.MotionMagicJerk = 400_tr_per_s_cu;
  // coral scoring motor configs
  configs::TalonFXSConfiguration scoringCfg{};
  scoringCfg.Commutation.MotorArrangement = signals::MotorArrangementValue::Minion_JST;
  scoringCfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  // algae scoring motor configs
  configs::TalonFXSConfiguration algaeScoringCfg{};
  algaeScoringCfg.Commutation.MotorArrangement = signals::MotorArrangementValue::Minion_JST;
  algaeScoringCfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  // apply the configurations
  Util::configureMotor(coralAngleMotor, coralAngleCfg);
  Util::configureMotor(algaeAngleMotor, algaeAngleCfg);
  Util::configureMotor(elevatorMotor, elevatorCfg);
  Util::configureMotor(scoringMotor, scoringCfg);
  Util::configureMotor(algaeScoringMotor, algaeScoringCfg);
}