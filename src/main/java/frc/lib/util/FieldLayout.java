package frc.lib.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class FieldLayout {
  public enum ReefPositions {
    A, B, C, D, E, F, G, H, I, J, K, L
  };

  public static HashMap<ReefPositions, Pose2d> reefPositionPose = new HashMap<>() {{ 
    put(ReefPositions.A, new Pose2d());
    put(ReefPositions.B, new Pose2d());
    put(ReefPositions.C, new Pose2d());
    put(ReefPositions.D, new Pose2d());
    put(ReefPositions.E, new Pose2d());
    put(ReefPositions.F, new Pose2d());
    put(ReefPositions.G, new Pose2d());
    put(ReefPositions.H, new Pose2d());
    put(ReefPositions.I, new Pose2d());
    put(ReefPositions.J, new Pose2d());
    put(ReefPositions.K, new Pose2d());
    put(ReefPositions.L, new Pose2d());
  }};

  public static ReefPositions findClosestReefPose() {
    Pose2d currentPose = Drive.getInstance().getPose();
    double shortestDistance = Double.POSITIVE_INFINITY;
    ReefPositions closestReefPosition = ReefPositions.A;

    for (Map.Entry<ReefPositions, Pose2d> entry : FieldLayout.reefPositionPose.entrySet()) {
      double currentDistance = Math.abs(currentPose.getTranslation().getDistance(entry.getValue().getTranslation()));

      if (currentDistance < shortestDistance) {
        shortestDistance = currentDistance;
        closestReefPosition = entry.getKey();
      }
    }

    return closestReefPosition;
  }
}
