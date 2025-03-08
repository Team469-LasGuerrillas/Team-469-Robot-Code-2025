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
  /* ORDER: Position (A, B, C, etc.); Side (Left, Right); Color (Blue, Red) */
  public enum ReefPositions {
    ALB, ARB, BLB, BRB, CLB, CRB, DLB, DRB, ELB, ERB, FLB, FRB,
    ALR, ARR, BLR, BRR, CLR, CRR, DLR, DRR, ELR, ERR, FLR, FRR
  };

  public static Transform2d LEFT_TRANSFORM = new Transform2d(-1.2, Units.inchesToMeters(6.5), new Rotation2d(Math.PI));
  public static Transform2d RIGHT_TRANSFORM = new Transform2d(-1.2, Units.inchesToMeters(-6.5), new Rotation2d(Math.PI));

  public static Transform2d L1_TRANSFORM = new Transform2d(0.5, 0, new Rotation2d());

  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static Pose2d REEF_CENTER_BLUE = new Pose2d(4.5, aprilTagFieldLayout.getFieldWidth() / 2, new Rotation2d());

  public static Pose2d REEF_CENTER_RED = new Pose2d(aprilTagFieldLayout.getFieldLength() - 4.5, aprilTagFieldLayout.getFieldWidth() / 2, new Rotation2d());

  public static HashMap<ReefPositions, Pose2d> reefPositionPoseRight = new HashMap<>() {{ 
    put(ReefPositions.ARB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.BRB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.CRB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.DRB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.ERB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.FRB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.ARR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.BRR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.CRR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.DRR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.ERR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.FRR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(RIGHT_TRANSFORM));
  }};

  public static HashMap<ReefPositions, Pose2d> reefPositionPoseLeft = new HashMap<>() {{
    put(ReefPositions.ALB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.BLB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.CLB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.DLB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.ELB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.FLB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.ALR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.BLR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.CLR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.DLR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.ELR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.FLR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(LEFT_TRANSFORM));
  }};

  public static ReefPositions findClosestReefPoseLeft() {
    return findClosestReefPose(reefPositionPoseLeft);
  }

  public static ReefPositions findClosestReefPoseRight() {
    return findClosestReefPose(reefPositionPoseRight);
  }

  private static ReefPositions findClosestReefPose(HashMap<ReefPositions, Pose2d> reefPositionPose) {
    Pose2d currentPose = Drive.getInstance().getPose();
    double shortestDistance = 999999999999999.0;
    ReefPositions closestReefPosition = ReefPositions.ALB;

    for (Map.Entry<ReefPositions, Pose2d> entry : reefPositionPose.entrySet()) {
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
