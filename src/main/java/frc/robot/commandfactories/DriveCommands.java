package frc.robot.commandfactories;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.lib.util.math.GeomUtil;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;
import frc.robot.subsystems.endEffectors.CoralWristEndEffector;

public class DriveCommands {
  public static Command acceptTeleopFieldOriented(CommandXboxController controller, boolean isDefault) {
    if (isDefault) {
      return Drive.getInstance().run(
        () ->
            Drive.getInstance().acceptTeleopInput(
                -controller.getLeftY(),
                -controller.getLeftX(),
                -controller.getRightX(),
                false));
    } else {
      return Commands.run(
        () ->
            Drive.getInstance().acceptTeleopInput(
                -controller.getLeftY(),
                -controller.getLeftX(),
                -controller.getRightX(),
                false));
    }
  }

  public static Command acceptTeleopRobotOriented(CommandXboxController controller, boolean isDefault) {
    {
      if (isDefault) {
        return Drive.getInstance().run(
          () ->
              Drive.getInstance().acceptTeleopInput(
                  -controller.getLeftY(),
                  -controller.getLeftX(),
                  -controller.getRightX(),
                  true));
      } else {
        return Commands.run(
          () ->
              Drive.getInstance().acceptTeleopInput(
                  -controller.getLeftY(),
                  -controller.getLeftX(),
                  -controller.getRightX(),
                  true));
      }
    }
  }

  public static Command autoRotate(CommandXboxController controller, Supplier<Rotation2d> heading) {
    return Commands.startEnd(
      () -> Drive.getInstance().setHeadingGoal(heading),
      () -> Drive.getInstance().clearMode()).alongWith(acceptTeleopFieldOriented(controller, false));
  }

  public static Command aimAssistToPose(CommandXboxController controller, Pose2d pose, double weight) {
    return Commands.startEnd(
    () -> Drive.getInstance().setAimAssist(pose, weight), 
    () -> Drive.getInstance().clearMode()).alongWith(acceptTeleopFieldOriented(controller, false));
  } 

  public static Command aimAssistToReefPose(CommandXboxController controller, ReefPositions position, double weight) {
    Pose2d pose = FieldLayout.reefPositionPoseRight.get(position);

    return aimAssistToPose(controller, pose, weight);
  }

  public static Command aimAssistToCoralLeft(CommandXboxController controller, double weight) {
    return Commands.deferredProxy(
      () -> aimAssistToReefPose(controller, FieldLayout.findClosestReefPoseLeft(), weight)
    );
  }

  public static Command aimAssistToCoralRight(CommandXboxController controller, double weight) {
    return Commands.deferredProxy(
      () -> aimAssistToReefPose(controller, FieldLayout.findClosestReefPoseRight(), weight)
    );
  }

  public static Command pidToPoint(Supplier<Pose2d> pose) {
    return Commands.startEnd(
      () -> Drive.getInstance().setPIDToPointGoal(pose),
      () -> Drive.getInstance().clearMode());
  }

  public static Command pidToPoint(Supplier<Pose2d> pose, boolean runFineStep) {
    return Commands.startEnd(
      () -> Drive.getInstance().setPIDToPointGoal(pose, runFineStep),
      () -> Drive.getInstance().clearMode());
  }

  public static Command pidToReefPose(ReefPositions position) {
    final Pose2d reefPose;

    if (CoralWristEndEffector.getInstance().getRequestedPosition() == CoralEndEffectorConstants.CORAL_L1_POS) {
      if (FieldLayout.reefPositionPoseLeft.containsKey(position)) {
        reefPose = FieldLayout.reefPositionToPose2d(position).transformBy(FieldLayout.TROUGH_TRANSFORM_LEFT);
      } else {
        reefPose = FieldLayout.reefPositionToPose2d(position).transformBy(FieldLayout.TROUGH_TRANSFORM_RIGHT);
      }
    } else {
        reefPose = FieldLayout.reefPositionToPose2d(position);
    }

    // return pidToPoint(() -> reefPose);
    final Transform2d newTransform;

    if (AlgaeWristEndEffector.getInstance().getRequestedPosition() == AlgaeEndEffectorConstants.ALGAE_WRIST_L2_L3) {
      newTransform = FieldLayout.ALGAE_TRANSFORM;
    } else {
      newTransform = FieldLayout.L1_TRANSFORM;
    }

    return Commands.sequence(
      Commands.deadline(
        Commands.waitUntil(
          () -> (
            Drive.getInstance().isOnTarget(FieldLayout.HOLDING_TOLERANCE_TRANSFORM, DriveConstants.L1_LINEAR_TOLERANCE_METERS, DriveConstants.L1_HEADING_TOLERANCE_DEGREES)
            && Elevator.getInstance().isCoralElevatorOnTarget(ElevatorConstants.IS_ON_TARGET_HUGE)
            && AlgaeWristEndEffector.getInstance().isOnTarget()
            )
        ),
        pidToPoint(() -> reefPose.transformBy(newTransform), false)
      ),
      pidToPoint(() -> reefPose, true)
    );
  }

  public static Command pidToClosestReefPoseLeft() {
    return Commands.deferredProxy(
      () -> pidToReefPose(FieldLayout.findClosestReefPoseLeft())
    );
  }

  public static Command pidToClosestReefPoseRight() {
    return Commands.deferredProxy(
      () -> pidToReefPose(FieldLayout.findClosestReefPoseRight())
    );
  }

  public static Command autoScoreToClosestReefPoseLeft() {
    return Commands.deferredProxy(
      () -> AutonCommands.driveAndAutoScore(FieldLayout.findClosestReefPoseLeft())
    );
  }

  public static Command autoScoreToClosestReefPoseRight() {
    return Commands.deferredProxy(
      () -> AutonCommands.driveAndAutoScore(FieldLayout.findClosestReefPoseRight())
    );
  }
}
