#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/ScoringMech.h"
#include "subsystems/Swerve.h"

namespace Score {
    frc2::CommandPtr ScoreCoral(Swerve &swerve, ScoringMech &scoringMech, const bool & isLeft);
    frc2::CommandPtr ScoreCoralForAuto(Swerve &swerve, ScoringMech &scoringMech, const bool & isLeft);
}

namespace GeneralCmds {
    frc2::CommandPtr HomeSafely(Swerve &swerve, ScoringMech &scoringMech);
    frc2::CommandPtr IntakeSafely(Swerve &swerve, ScoringMech &scoringMech);
}