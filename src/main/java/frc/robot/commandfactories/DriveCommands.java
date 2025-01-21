package frc.robot.commandfactories;

import java.util.Map;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.ReefPositions;
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

  // public static Command aimAssistToPose(CommandXboxController controller, Pose2d pose) {
  //   return Drive.getInstance().setAimAssist(() -> {return pose;}, () -> 0.0);
  // } 

  // public static Command pathfindToPose(Pose2d pose) {
  //   return Drive.getInstance().findPath(pose);
  // }

  // public static Command pathfindToReefPose(ReefPositions position) {
  //   Pose2d pose = FieldLayout.reefPositionPose.get(position);

  //   return pathfindToPose(pose);
  // }

  // public static Command pathfindToClosestReefPose() {
  //   ReefPositions closestReefPosition = FieldLayout.findClosestReefPose();

  //   return pathfindToReefPose(closestReefPosition);
  // }
}
