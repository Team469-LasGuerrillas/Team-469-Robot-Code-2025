package frc.robot.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldLayout;
import frc.lib.util.Station;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffectors.CoralIntakeEndEffector;

public class AutonCommands {
  public static Command driveAndScoreL4ToReefPosition(ReefPositions reefPosition) {
    return Commands.sequence(
      Commands.deadline(
        Commands.waitUntil(
          () -> 
            Drive.getInstance().isOnTarget(
              reefPosition, 
              DriveConstants.LINEAR_TOLERACE_TO_SCORE_METERS, 
              DriveConstants.HEADING_TOLERANCE_TO_SCORE_DEGREES,
              25)
            && Elevator.getInstance().isOnTarget()), 
        DriveCommands.pidToReefPose(reefPosition),
        Commands.sequence(
          Commands.waitUntil(
            () -> Drive.getInstance().isOnTarget(
              reefPosition, 
              DriveConstants.LINEAR_TOLERANCE_TO_RAISE_ELEVATOR, 
              DriveConstants.HEADING_TOLERANCE_TO_RAISE_ELEVATOR)),
          GlobalCommands.coralL4()
        )
      ),
      GlobalCommands.coralRelease()
    );
  }

  public static Command driveAndIntakeFromHumanPlayer() {
    Pose2d targetHPPose;
    if (Station.isRed()) targetHPPose = FieldLayout.HUMAN_PLAYER_RED;
    else targetHPPose = FieldLayout.HUMAN_PLAYER_BLUE;

    return Commands.deadline(
      Commands.waitUntil(
        () -> CoralIntakeEndEffector.getInstance().hasCoral()),
      DriveCommands.pidToPoint(() -> targetHPPose),
        GlobalCommands.humanPlayerIntake()
    );
  }
}
