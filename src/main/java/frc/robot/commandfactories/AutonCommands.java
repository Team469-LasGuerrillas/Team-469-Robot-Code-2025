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
            // Elevator.getInstance().isCoralOnTarget() && This is a hacky fix to make elevator come down for some reason
            Drive.getInstance().isOnTarget(
              reefPosition, 
              DriveConstants.LINEAR_TOLERACE_TO_SCORE_METERS, 
              DriveConstants.HEADING_TOLERANCE_TO_SCORE_DEGREES,
              10)
            ),
        DriveCommands.pidToReefPose(reefPosition), // Drive to reef
        Commands.sequence(
          Commands.waitUntil( // When we are "approximate" to the reef...
            () -> Drive.getInstance().isOnTarget(
              reefPosition, 
              DriveConstants.LINEAR_TOLERANCE_TO_RAISE_ELEVATOR, 
              DriveConstants.HEADING_TOLERANCE_TO_RAISE_ELEVATOR)),
          GlobalCommands.coralL4NoAlgae() // Raise the elevator
        )
      ), // End deadline group (at this point we should be in scoring position)

      Commands.deadline(
        Commands.waitSeconds(0.5), //Do the following for 0.75 seconds
        GlobalCommands.coralRelease(), // Score the coral
        GlobalCommands.coralL4NoAlgae()) // Keep the elevator up
    );
  }

  public static Command driveAndIntakeFromHumanPlayerLeft() {
    System.out.println("The hp intake command has run.");
    Pose2d targetHPPose;
    if (Station.isRed()) targetHPPose = FieldLayout.HUMAN_PLAYER_RED_LEFT;
    else targetHPPose = FieldLayout.HUMAN_PLAYER_BLUE_LEFT;

    return Commands.deadline(
      Commands.waitUntil(
        () -> CoralIntakeEndEffector.getInstance().hasCoral()), // Run this group until we have a coral
      DriveCommands.pidToPoint(() -> targetHPPose), // Drive to HP station
      GlobalCommands.humanPlayerIntake() // Put elevator in HP load position
    );
  }

  public static Command driveAndIntakeFromHumanPlayerRight() {
    System.out.println("The hp intake command has run.");
    Pose2d targetHPPose;
    if (Station.isRed()) targetHPPose = FieldLayout.HUMAN_PLAYER_RED_RIGHT;
    else targetHPPose = FieldLayout.HUMAN_PLAYER_BLUE_RIGHT;

    return Commands.deadline(
      Commands.waitUntil(
        () -> CoralIntakeEndEffector.getInstance().hasCoral()), // Run this group until we have a coral
      DriveCommands.pidToPoint(() -> targetHPPose), // Drive to HP station
      GlobalCommands.humanPlayerIntake() // Put elevator in HP load position
    );
  }
}
