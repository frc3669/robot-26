#include <frc2/command/Commands.h>
#include "commands/MultiSubsystem.h"

frc2::CommandPtr Score::ScoreCoral(Swerve &swerve, ScoringMech &scoringMech, const bool & isLeft) {
    return frc2::cmd::Race(
            frc2::cmd::Sequence(
                frc2::cmd::Parallel(
                    swerve.driveToPoleIntermediate(isLeft),
                    scoringMech.goToCoralScoringPosition().OnlyIf([&swerve] { return swerve.safeToMoveCoralManipulator(); })),
                scoringMech.goToCoralScoringPosition(),
                swerve.driveToPole(isLeft),
                scoringMech.ejectCoral()),
            frc2::cmd::Wait(5_s))
        .OnlyIf([&swerve] { return swerve.reefWithinRange(); })
        .WithName("Score Right");
}

frc2::CommandPtr Score::ScoreCoralForAuto(Swerve &swerve, ScoringMech &scoringMech, const bool & isLeft) {
    return frc2::cmd::Race(
            frc2::cmd::Sequence(
                swerve.driveToPole(isLeft),
                scoringMech.ejectCoral()),
            frc2::cmd::Wait(5_s))
        .WithName("Score Right");
}

frc2::CommandPtr GeneralCmds::HomeSafely(Swerve &swerve, ScoringMech &scoringMech) {
    return scoringMech.home().OnlyIf([&swerve] { return swerve.safeToMoveCoralManipulator(); }).WithName("homing if safe");
}

frc2::CommandPtr GeneralCmds::IntakeSafely(Swerve &swerve, ScoringMech &scoringMech) {
    return scoringMech.intake().OnlyIf([&swerve] { return swerve.safeToMoveCoralManipulator(); }).WithName("intaking if safe");
}