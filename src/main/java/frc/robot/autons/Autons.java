package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.AutoScore;
import frc.lib.util.FieldLayout;
import frc.lib.util.Station;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.robot.commandfactories.AutonCommands;
import frc.robot.commandfactories.DriveCommands;

public class Autons {
  public static Command centerOnePiece() {
    ReefPositions targetReefPosition = ReefPositions.DLB;
    if (Station.isRed()) targetReefPosition = ReefPositions.DLR;
    
    AutoScore.resetAutoScoreToL4();

    return Commands.sequence(
      AutonCommands.driveAndAutoScoreInAuton(targetReefPosition)
    );
  }

  public static Command centerOnePiecePlusAlgae() {
    ReefPositions targetReefPosition;
    if (Station.isRed()) targetReefPosition = ReefPositions.DLR;
    else targetReefPosition = ReefPositions.DLB;
        
    AutoScore.resetAutoScoreToL4();

    return Commands.sequence(
      AutonCommands.driveAndAutoScoreInAuton(targetReefPosition, true), // Coral L4
      Commands.runOnce(() -> AutoScore.resetAutoScoreToL3()),
      AutonCommands.driveAndAutoScoreL3InAuton(targetReefPosition, true),
      AutonCommands.scoreAlgaeInBarge()
    );
  }

  public static Command threePieceLeft() {
    ReefPositions EL;
    if (Station.isRed()) EL = ReefPositions.ERR;
    else EL = ReefPositions.ERB;

    ReefPositions FL;
    if (Station.isRed()) FL = ReefPositions.FLR;
    else FL = ReefPositions.FLB;

    ReefPositions FR;
    if (Station.isRed()) FR = ReefPositions.FRR;
    else FR = ReefPositions.FRB;

    ReefPositions AL;
    if (Station.isRed()) AL = ReefPositions.ALR;
    else AL = ReefPositions.ALB;

    AutoScore.resetAutoScoreToL4();
  
    return 
    Commands.sequence(
      AutonCommands.driveAndAutoScoreInAuton(EL),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      AutonCommands.driveAndAutoScoreInAuton(FL),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      AutonCommands.driveAndAutoScoreInAuton(FR),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      AutonCommands.driveAndAutoScoreInAuton(AL)
    );
  }

  public static Command threePieceRight() {
    ReefPositions CR;
    if (Station.isRed()) CR = ReefPositions.CLR;
    else CR = ReefPositions.CLB;

    ReefPositions BR;
    if (Station.isRed()) BR = ReefPositions.BRR;
    else BR = ReefPositions.BRB;

    ReefPositions BL;
    if (Station.isRed()) BL = ReefPositions.BLR;
    else BL = ReefPositions.BLB;

    ReefPositions AR;
    if (Station.isRed()) AR = ReefPositions.ARR;
    else AR = ReefPositions.ARB;

    AutoScore.resetAutoScoreToL4();

    return Commands.sequence(
      AutonCommands.driveAndAutoScoreInAuton(CR),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      AutonCommands.driveAndAutoScoreInAuton(BR),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      AutonCommands.driveAndAutoScoreInAuton(BL),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      AutonCommands.driveAndAutoScoreInAuton(AR)
    );
  }
}