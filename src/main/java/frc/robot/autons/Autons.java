package frc.robot.autons;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public static Command startEFF() {
    ReefPositions EL = ReefPositions.ELB;
    if (Station.isRed()) EL = ReefPositions.ELR;
    ReefPositions FL = ReefPositions.FLB;
    if (Station.isRed()) FL = ReefPositions.FLR;
    ReefPositions FR = ReefPositions.FRB;
    if (Station.isRed()) FR = ReefPositions.FRR;

    return Commands.sequence(
      AutonCommands.driveAndScoreL4ToReefPosition(EL),
      AutonCommands.driveAndIntakeFromHumanPlayer(),
      AutonCommands.driveAndScoreL4ToReefPosition(FL),
      AutonCommands.driveAndIntakeFromHumanPlayer(),
      AutonCommands.driveAndScoreL4ToReefPosition(FR)
    );
  }
}