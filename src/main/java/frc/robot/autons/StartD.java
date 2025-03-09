package frc.robot.autons;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.Station;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.robot.commandfactories.AutonCommands;

public class StartD {
  public static Command startD() {
    ReefPositions targetReefPosition = ReefPositions.DLB;
    if (Station.isRed()) targetReefPosition = ReefPositions.DLR;
    
    return Commands.sequence(
      AutonCommands.driveAndScoreL4ToReefPosition(targetReefPosition)
    );
  }
}