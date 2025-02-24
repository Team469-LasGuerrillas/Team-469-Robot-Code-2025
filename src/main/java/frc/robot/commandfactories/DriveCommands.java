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

  public static Command pidToPoint(Supplier<Pose2d> pose) {
    return Commands.startEnd(
      () -> Drive.getInstance().setPIDToPointGoal(pose),
      () -> Drive.getInstance().clearMode());
  }

  public static Command aimAssistToPose(CommandXboxController controller, Pose2d pose, double weight) {
    return Commands.startEnd(
    () -> Drive.getInstance().setAimAssist(pose, weight), 
    () -> Drive.getInstance().clearMode()).alongWith(acceptTeleopFieldOriented(controller, false));
  } 

  public static Command aimAssistToReefPose(CommandXboxController controller, ReefPositions position, double weight) {
    Pose2d pose = FieldLayout.reefPositionPose.get(position);

    return aimAssistToPose(controller, pose, weight);
  }

  public static Command aimAssistToClosestReefPose(CommandXboxController controller, double weight) {
    return Commands.deferredProxy(
      () -> aimAssistToReefPose(controller, FieldLayout.findClosestReefPose(), weight)
    );
  }

  public static Command aimAssistToCoral(CommandXboxController controller, double weight) {
    return Commands.deferredProxy(
      () -> aimAssistToPose(controller, null, weight)
    );
  }

  public static Command pathfindToPose(Pose2d pose) {
    return Commands.startEnd(
      () -> Drive.getInstance().setPathfinding(pose), 
      () -> Drive.getInstance().clearMode());
  }

  public static Command pathfindToReefPose(ReefPositions position) {
    Pose2d pose = FieldLayout.reefPositionPose.get(position);

    return pathfindToPose(pose);
  }

  public static Command pathfindToClosestReefPose() {
    ReefPositions closestReefPosition = FieldLayout.findClosestReefPose();

    return pathfindToReefPose(closestReefPosition);
  }

  public static Command pidToReefPose(ReefPositions position) {
    Pose2d pose = FieldLayout.reefPositionPose.get(position);

    return pidToPoint(() -> pose);
  }

  public static Command pidToClosestReefPose() {
    return Commands.deferredProxy(
      () -> pidToReefPose(FieldLayout.findClosestReefPose())
    );
  }
}
