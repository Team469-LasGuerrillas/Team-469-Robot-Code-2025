package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.Station;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.robot.commandfactories.AutonCommands;

public class Autons {
  public static Command startD() {
    ReefPositions targetReefPosition = ReefPositions.DLB;
    if (Station.isRed()) targetReefPosition = ReefPositions.DLR;
    
    return Commands.sequence(
      AutonCommands.driveAndScoreL4ToReefPosition(targetReefPosition)
    );
  }

  public static Command threePieceLeft() {
    ReefPositions EL = ReefPositions.ELB;
    if (Station.isRed()) EL = ReefPositions.ELR;
    ReefPositions FL = ReefPositions.FLB;
    if (Station.isRed()) FL = ReefPositions.FLR;
    ReefPositions FR = ReefPositions.FRB;
    if (Station.isRed()) FR = ReefPositions.FRR;
    ReefPositions AL = ReefPositions.ALB;
    if (Station.isRed()) FR = ReefPositions.ALR;

    return Commands.sequence(
      AutonCommands.driveAndScoreL4ToReefPosition(EL),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      AutonCommands.driveAndScoreL4ToReefPosition(FL),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      AutonCommands.driveAndScoreL4ToReefPosition(FR),
      AutonCommands.driveAndIntakeFromHumanPlayerLeft(),
      AutonCommands.driveAndScoreL4ToReefPosition(AL)
    );
  }

  public static Command threePieceRight() {
    ReefPositions CR = ReefPositions.CRB;
    if (Station.isRed()) CR = ReefPositions.CRR;
    ReefPositions BR = ReefPositions.BRB;
    if (Station.isRed()) BR = ReefPositions.BRR;
    ReefPositions BL = ReefPositions.BLB;
    if (Station.isRed()) BL = ReefPositions.BLR;
    ReefPositions AR = ReefPositions.ARB;
    if (Station.isRed()) AR = ReefPositions.ARR;

    return Commands.sequence(
      AutonCommands.driveAndScoreL4ToReefPosition(CR),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      AutonCommands.driveAndScoreL4ToReefPosition(BR),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      AutonCommands.driveAndScoreL4ToReefPosition(BL),
      AutonCommands.driveAndIntakeFromHumanPlayerRight(),
      AutonCommands.driveAndScoreL4ToReefPosition(AR)
    );
  }
}