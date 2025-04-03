package frc.robot.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoScore;
import frc.lib.util.FieldLayout;
import frc.lib.util.Station;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.AutonConstants;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffectors.CoralIntakeEndEffector;
import frc.robot.subsystems.endEffectors.CoralWristEndEffector;
import frc.robot.subsystems.endEffectors.AlgaeIntakeEndEffector;
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;

public class AutonCommands {
  public static Command driveAndAutoScore(ReefPositions reefPosition) {
    return Commands.deferredProxy(() -> 
      Commands.sequence(
        Commands.deadline(
          Commands.waitUntil(
            () -> 
              Elevator.getInstance()
                .isCoralElevatorOnTarget(ElevatorConstants.NUM_OF_ON_TARGET_LOOPS) 
              && CoralWristEndEffector.getInstance()
                .isCoralWristOnTarget(CoralEndEffectorConstants.NUM_OF_ON_TARGET_LOOPS) 
              && Drive.getInstance()
                .isOnTarget(
                  reefPosition, 
                  DriveConstants.LINEAR_TOLERACE_TO_SCORE_METERS, 
                  DriveConstants.HEADING_TOLERANCE_TO_SCORE_DEGREES,
                  AutonConstants.NUM_OF_ON_TARGET_LOOPS
                )
          ),
          DriveCommands.pidToReefPose(reefPosition), // Drive to reef
          CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_FINAL_RETAINING_VOLTAGE),
          CoralEndEffectorCommands.coralWrist(AutoScore.getNextCoralWristPos()),
          Commands.sequence(
            Commands.waitUntil( // When we are "approximate" to the reef...
              () -> Drive.getInstance().isOnTarget(
                reefPosition, 
                DriveConstants.LINEAR_TOLERANCE_TO_RAISE_ELEVATOR, 
                DriveConstants.HEADING_TOLERANCE_TO_RAISE_ELEVATOR)
            ),
            Commands.parallel(
              AlgaeEndEffectorCommands.algaeIntake((AutoScore.getNextAlgaeIntakeVol())),
              AlgaeEndEffectorCommands.algaeWrist(AutoScore.getNextAlgaeWristPos()),
              ElevatorCommands.setTargetPosFromZero(
                AutoScore.getNextCoralElevatorPos(),
                AutoScore.getNextAlgaeElevatorPos()
              )
            )
          )
        ),
        Commands.deadline(
          GlobalCommands.coralRelease(), // Score the coral
          CoralEndEffectorCommands.coralWrist(AutoScore.getNextCoralWristPos()),
          AlgaeEndEffectorCommands.algaeIntake((AutoScore.getNextAlgaeIntakeVol())),
          AlgaeEndEffectorCommands.algaeWrist(AutoScore.getNextAlgaeWristPos()),
          ElevatorCommands.setTargetPosFromZero(
            AutoScore.getNextCoralElevatorPos(),
            AutoScore.getNextAlgaeElevatorPos()
          )
        ),
        Commands.parallel(
          DriveCommands.pidToPoint(
            () -> FieldLayout.reefPositionToPose2d(reefPosition).transformBy(FieldLayout.L2_TRANSFORM)
          ),
          CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_DEFAULT_VOLTAGE),
          CoralEndEffectorCommands.coralWrist(AutoScore.getNextCoralWristPos()),
          AlgaeEndEffectorCommands.algaeIntake((AutoScore.getNextAlgaeIntakeVol())),
          AlgaeEndEffectorCommands.algaeWrist(AutoScore.getNextAlgaeWristPos()),
          ElevatorCommands.setTargetPosFromZero(
            AutoScore.getNextCoralElevatorPos(),
            AutoScore.getNextAlgaeElevatorPos()
          )
        )
      )
    );
  }

  public static Command driveAndAutoScoreInAuton(ReefPositions reefPosition) {
    return Commands.sequence(
        Commands.race(
          Commands.waitSeconds(AutonConstants.TIME_EXCEED),
          Commands.deadline(
            Commands.waitUntil(
              () -> 
                Elevator.getInstance()
                  .isCoralElevatorOnTarget(ElevatorConstants.NUM_OF_ON_TARGET_LOOPS) 
                && CoralWristEndEffector.getInstance()
                  .isCoralWristOnTarget(CoralEndEffectorConstants.NUM_OF_ON_TARGET_LOOPS) 
                && Drive.getInstance()
                  .isOnTarget(
                    reefPosition, 
                    DriveConstants.LINEAR_TOLERACE_TO_SCORE_METERS, 
                    DriveConstants.HEADING_TOLERANCE_TO_SCORE_DEGREES,
                    AutonConstants.NUM_OF_ON_TARGET_LOOPS
                  )
            ),
            DriveCommands.pidToReefPose(reefPosition), // Drive to reef
            CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_FINAL_RETAINING_VOLTAGE),
            CoralEndEffectorCommands.coralWrist(AutoScore.getNextCoralWristPos()),
            Commands.sequence(
              Commands.waitUntil( // When we are "approximate" to the reef...
                () -> Drive.getInstance().isOnTarget(
                  reefPosition, 
                  DriveConstants.LINEAR_TOLERANCE_TO_RAISE_ELEVATOR, 
                  DriveConstants.HEADING_TOLERANCE_TO_RAISE_ELEVATOR)
              ),
              Commands.parallel(
                AlgaeEndEffectorCommands.algaeIntake((AutoScore.getNextAlgaeIntakeVol())),
                AlgaeEndEffectorCommands.algaeWrist(AutoScore.getNextAlgaeWristPos()),
                ElevatorCommands.setTargetPosFromZero(
                  AutoScore.getNextCoralElevatorPos(),
                  AutoScore.getNextAlgaeElevatorPos()
                )
              )
            )
          )
        ),
        Commands.deadline(
          GlobalCommands.coralRelease(), // Score the coral
          CoralEndEffectorCommands.coralWrist(AutoScore.getNextCoralWristPos()),
          AlgaeEndEffectorCommands.algaeIntake((AutoScore.getNextAlgaeIntakeVol())),
          AlgaeEndEffectorCommands.algaeWrist(AutoScore.getNextAlgaeWristPos()),
          ElevatorCommands.setTargetPosFromZero(
            AutoScore.getNextCoralElevatorPos(),
            AutoScore.getNextAlgaeElevatorPos()
          )
        )
      );
  }

  public static Command driveAndScoreL4ToReefPosition(ReefPositions reefPosition) {
    return Commands.sequence(
      Commands.race(
        Commands.waitSeconds(AutonConstants.TIME_EXCEED),
        Commands.deadline(
          Commands.waitUntil(
            () -> 
              // Elevator.getInstance().isCoralOnTarget() && This is a hacky fix to make elevator come down for some reason
              Drive.getInstance().isOnTarget(
                reefPosition, 
                DriveConstants.LINEAR_TOLERACE_TO_SCORE_METERS, 
                DriveConstants.HEADING_TOLERANCE_TO_SCORE_DEGREES,
                AutonConstants.NUM_OF_ON_TARGET_LOOPS)
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
        ) // End deadline group (at this point we should be in scoring position)
      ),
      Commands.deadline(
        Commands.waitSeconds(AutonConstants.CORAL_RELEASE_TIME), //Do the following for 0.5 seconds
        GlobalCommands.coralRelease(), // Score the coral
        GlobalCommands.coralL4NoAlgae()) // Keep the elevator up
    );
  }
      
  public static Command descoreAlgaeFromReefPosition(ReefPositions reefPosition) {
    Pose2d transformedReefPose = FieldLayout.reefPositionToPose2d(reefPosition).transformBy(FieldLayout.ALGAE_TRANSFORM);
    
    return Commands.sequence(
      Commands.deadline( // Drive back from reef and lower elevator
        Commands.waitUntil(
          () -> Drive.getInstance().isOnTarget(
              transformedReefPose, 
              DriveConstants.LINEAR_TOLERACE_TO_SCORE_METERS, 
              DriveConstants.HEADING_TOLERANCE_TO_SCORE_DEGREES)
          && Elevator.getInstance().isCoralElevatorOnTarget()
          && Elevator.getInstance().isAlgaeElevatorOnTarget()
        ),
        DriveCommands.pidToPoint(() -> transformedReefPose),
        GlobalCommands.coralL3NoAlgae() 
      ),
      Commands.deadline( // Set Algae Position
        Commands.waitUntil(
          () -> AlgaeWristEndEffector.getInstance().isOnTarget()
          && Elevator.getInstance().isCoralElevatorOnTarget()
          && Elevator.getInstance().isAlgaeElevatorOnTarget()
        ), 
        GlobalCommands.coralL3()
      ),
      Commands.deadline( // Drive back to reef
        Commands.waitUntil(
          () -> Drive.getInstance().isOnTarget(
              reefPosition, 
              DriveConstants.LINEAR_TOLERACE_TO_SCORE_METERS, 
              DriveConstants.HEADING_TOLERANCE_TO_SCORE_DEGREES)
          && Elevator.getInstance().isCoralElevatorOnTarget()
          && Elevator.getInstance().isAlgaeElevatorOnTarget()
        ),
        DriveCommands.pidToReefPose(reefPosition),
        GlobalCommands.coralL3() 
      )
    );
  }

  public static Command scoreAlgaeInBarge() {
    Pose2d bargePose;
    if (Station.isRed()) bargePose = FieldLayout.BARGE_POSITION_RED;
    else bargePose = FieldLayout.BARGE_POSITION_BLUE;

    return Commands.sequence(
      Commands.deadline( // Drive to barge
        Commands.waitUntil(
          () -> Drive.getInstance().isOnTarget(
            bargePose, 
            DriveConstants.LINEAR_TOLERACE_TO_SCORE_METERS, 
            DriveConstants.HEADING_TOLERANCE_TO_SCORE_DEGREES)
        ),
        DriveCommands.pidToPoint(() -> bargePose)
      ),
      Commands.deadline( // Raise elevator
        Commands.waitUntil(
          () -> Elevator.getInstance().isCoralElevatorOnTarget()
          && Elevator.getInstance().isAlgaeElevatorOnTarget()
        ),
        GlobalCommands.algaeBarge()
      ),
      GlobalCommands.algaeRelease() // Score algae
    );
  }

  public static Command driveAndIntakeFromHumanPlayerLeft() {
    Pose2d targetHPPose;
    if (Station.isRed()) targetHPPose = FieldLayout.HUMAN_PLAYER_RED_LEFT;
    else targetHPPose = FieldLayout.HUMAN_PLAYER_BLUE_LEFT;

    return
      Commands.deadline(
        Commands.waitUntil(
          () -> CoralIntakeEndEffector.getInstance().hasCoral()), // Run this group until we have a coral
        DriveCommands.pidToPoint(() -> targetHPPose, false), // Drive to HP station
        GlobalCommands.humanPlayerIntake(), // Put elevator in HP load position
        AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE),
        AlgaeEndEffectorCommands.algaeWristDefault(),
        ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_DEFAULT_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)
      );
}

  public static Command driveAndIntakeFromHumanPlayerRight() {
    Pose2d targetHPPose;
    if (Station.isRed()) targetHPPose = FieldLayout.HUMAN_PLAYER_RED_RIGHT;
    else targetHPPose = FieldLayout.HUMAN_PLAYER_BLUE_RIGHT;

    return 
      Commands.deadline(
        Commands.waitUntil(
          () -> CoralIntakeEndEffector.getInstance().hasCoral()), // Run this group until we have a coral
        DriveCommands.pidToPoint(() -> targetHPPose, false), // Drive to HP station
        GlobalCommands.humanPlayerIntake(), // Put elevator in HP load position
        AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE),
        AlgaeEndEffectorCommands.algaeWristDefault(),
        ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_DEFAULT_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)
      );
  }
}
