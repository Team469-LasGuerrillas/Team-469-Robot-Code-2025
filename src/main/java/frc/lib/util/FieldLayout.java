package frc.lib.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.math.GeomUtil;
import frc.robot.subsystems.drive.Drive;

public class FieldLayout {
  public enum ReefPositions {
    AL, AR, BL, BR, CL, CR, DL, DR, EL, ER, FL, FR
  };

  public static Transform2d LEFT_TRANSFORM = new Transform2d(-1.25, Units.inchesToMeters(6.5), new Rotation2d());
  public static Transform2d RIGHT_TRANSFORM = new Transform2d(-1.25, Units.inchesToMeters(-6.5), new Rotation2d());

  public static Pose2d REEF_CENTER = new Pose2d(4.5, AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getFieldWidth() / 2, new Rotation2d());

  public static HashMap<ReefPositions, Pose2d> reefPositionPose = new HashMap<>() {{ 
    put(ReefPositions.AL, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.AR, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.BL, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.BR, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.CL, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.CR, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.DL, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.DR, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.EL, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.ER, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.FL, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.FR, REEF_CENTER.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(RIGHT_TRANSFORM));
  }};

  public static ReefPositions findClosestReefPose() {
    Pose2d currentPose = Drive.getInstance().getPose();
    double shortestDistance = 999999999999999.0;
    ReefPositions closestReefPosition = ReefPositions.AL;

    for (Map.Entry<ReefPositions, Pose2d> entry : FieldLayout.reefPositionPose.entrySet()) {
      double currentDistance = Math.abs(currentPose.getTranslation().getDistance(entry.getValue().getTranslation()));

      if (currentDistance < shortestDistance) {
        shortestDistance = currentDistance;
        closestReefPosition = entry.getKey();
      }
    }

    System.out.println("Closest Reef Pose: " + closestReefPosition);
    return closestReefPosition;
  }
}
