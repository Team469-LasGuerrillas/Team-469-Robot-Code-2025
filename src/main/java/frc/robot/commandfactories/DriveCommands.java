package frc.robot.commandfactories;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.lib.util.math.GeomUtil;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

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

  public static Command pidToReefPose(ReefPositions position) {
    Pose2d poseRight = FieldLayout.reefPositionPoseRight.get(position);
    Pose2d poseLeft = FieldLayout.reefPositionPoseLeft.get(position);
    Pose2d realPose;

    if (poseRight != null) {
      realPose = poseRight;
    } else {
      realPose = poseLeft;
    }

    return pidToPoint(() -> realPose);

    // return Commands.sequence(
    //   Commands.deadline(
    //     Commands.waitUntil(
    //       () -> Drive.getInstance().isOnTarget(DriveConstants.L1_LINEAR_TOLERANCE_METERS, DriveConstants.L1_HEADING_TOLERANCE_DEGREES)
    //     ),
    //     pidToPoint(() -> realPose.transformBy(FieldLayout.L1_TRANSFORM))
    //   ),
    //   pidToPoint(() -> realPose)
    // );
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

  public static Command autoScoreToReefPoseLeft() {
    return Commands.deferredProxy(
      () -> AutonCommands.driveAndAutoScore(FieldLayout.findClosestReefPoseLeft())
    );
  }

  public static Command autoScoreToReefPoseRight() {
    return Commands.deferredProxy(
      () -> AutonCommands.driveAndAutoScore(FieldLayout.findClosestReefPoseRight())
    );
  }
}
