package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoScore;
import frc.lib.util.Station;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.robot.commandfactories.AutonCommands;

public class Autons {
  public static Command centerOnePiece() {
    // ReefPositions targetReefPosition = ReefPositions.DLB;
    // if (Station.isRed()) targetReefPosition = ReefPositions.DLR;
    
    AutoScore.resetAutoScoreToL4();

    return Commands.sequence(
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.DLR),
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.DLB), 
        Station::isRed
      )
    );
  }

  public static Command centerOnePiecePlusAlgae() {
    // ReefPositions targetReefPosition;
    // if (Station.isRed()) targetReefPosition = ReefPositions.DLR;
    // else targetReefPosition = ReefPositions.DLB;
        
    AutoScore.resetAutoScoreToL4();

    return Commands.sequence(
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.DLR, true), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.DLB, true), 
        Station::isRed
      ),
      Commands.runOnce(() -> AutoScore.resetAutoScoreToL3()),
      Commands.either(
        AutonCommands.driveAndAutoScoreL3InAuton(ReefPositions.DLR, true), 
        AutonCommands.driveAndAutoScoreL3InAuton(ReefPositions.DLB, true), 
        Station::isRed
      ),
      AutonCommands.scoreAlgaeInBarge()
    );

    // return Commands.sequence(
    //   AutonCommands.driveAndAutoScoreInAuton(targetReefPosition, true), // Coral L4
    //   Commands.runOnce(() -> AutoScore.resetAutoScoreToL3()),
    //   AutonCommands.driveAndAutoScoreL3InAuton(targetReefPosition, true),
    //   AutonCommands.scoreAlgaeInBarge()
    // );
  }

  public static Command threePieceLeft() {
    // ReefPositions EL;
    // if (Station.isRed()) EL = ReefPositions.ERR;
    // else EL = ReefPositions.ERB;

    // ReefPositions FL;
    // if (Station.isRed()) FL = ReefPositions.FLR;
    // else FL = ReefPositions.FLB;

    // ReefPositions FR;
    // if (Station.isRed()) FR = ReefPositions.FRR;
    // else FR = ReefPositions.FRB;

    // ReefPositions AL;
    // if (Station.isRed()) AL = ReefPositions.ALR;
    // else AL = ReefPositions.ALB;

    AutoScore.resetAutoScoreToL4();
  
    return 
    Commands.sequence(
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.ERR), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.ERB), 
        Station::isRed),
      // AutonCommands.driveAndAutoScoreInAuton(EL),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.FLR), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.FLB), 
        Station::isRed),
      // AutonCommands.driveAndAutoScoreInAuton(FL),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.FRR), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.FRB), 
        Station::isRed),
        // AutonCommands.driveAndAutoScoreInAuton(FR),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.ALR), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.ALB), 
        Station::isRed)
        // AutonCommands.driveAndAutoScoreInAuton(AL)
    );
  }

  public static Command threePieceRight() {
    // ReefPositions CR;
    // if (Station.isRed()) CR = ReefPositions.CLR;
    // else CR = ReefPositions.CLB;

    // ReefPositions BR;
    // if (Station.isRed()) BR = ReefPositions.BRR;
    // else BR = ReefPositions.BRB;

    // ReefPositions BL;
    // if (Station.isRed()) BL = ReefPositions.BLR;
    // else BL = ReefPositions.BLB;

    // ReefPositions AR;
    // if (Station.isRed()) AR = ReefPositions.ARR;
    // else AR = ReefPositions.ARB;

    AutoScore.resetAutoScoreToL4();

    return Commands.sequence(
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.CLR), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.CLB), 
        Station::isRed),
      // AutonCommands.driveAndAutoScoreInAuton(CR),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.BRR), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.BRB), 
        Station::isRed),
      // AutonCommands.driveAndAutoScoreInAuton(BR),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.BLR), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.BLB), 
        Station::isRed),
      // AutonCommands.driveAndAutoScoreInAuton(BL),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      Commands.either(
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.ARR), 
        AutonCommands.driveAndAutoScoreInAuton(ReefPositions.ARB), 
        Station::isRed)
      // AutonCommands.driveAndAutoScoreInAuton(AR)
    );
  }
}