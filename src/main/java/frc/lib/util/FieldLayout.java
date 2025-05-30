package frc.lib.util;

import java.util.HashMap;
import java.util.Map;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.math.GeomUtil;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class FieldLayout {
  /* ORDER: Position (A, B, C, etc.); Side (Left, Right); Color (Blue, Red) */
  public enum ReefPositions {
    ALB, ARB, BLB, BRB, CLB, CRB, DLB, DRB, ELB, ERB, FLB, FRB,
    ALR, ARR, BLR, BRR, CLR, CRR, DLR, DRR, ELR, ERR, FLR, FRR
  };

  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double FIELD_WIDTH = aprilTagFieldLayout.getFieldWidth(); // Y
  public static final double FIELD_LENGTH = aprilTagFieldLayout.getFieldLength(); // X

  public static final Pose2d HUMAN_PLAYER_BLUE_LEFT = new Pose2d(1.5, 7.35, Rotation2d.fromDegrees(130));
  public static final Pose2d HUMAN_PLAYER_RED_LEFT = new Pose2d(FIELD_LENGTH - 1.5, FIELD_WIDTH - 7.35, Rotation2d.fromDegrees(130 + 180));

  public static final Pose2d HUMAN_PLAYER_BLUE_RIGHT = new Pose2d(1.5, FIELD_WIDTH - 7.35, Rotation2d.fromDegrees(-130));
  public static final Pose2d HUMAN_PLAYER_RED_RIGHT = new Pose2d(FIELD_LENGTH - 1.5, 7.35, Rotation2d.fromDegrees(-130 + 180));

  public static final Pose2d BARGE_POSITION_RED = new Pose2d(FIELD_LENGTH - 7.2, FIELD_WIDTH - 5.3, Rotation2d.fromDegrees(0));
  public static final Pose2d BARGE_POSITION_BLUE = new Pose2d(7.2, 5.3, Rotation2d.fromDegrees(180));

  public static final Pose2d PROCESSOR_BLUE = new Pose2d(5.9, 0.75, new Rotation2d(0.5 * Math.PI));
  public static final Pose2d PROCESSOR_RED = new Pose2d(FIELD_LENGTH - 5.9, FIELD_WIDTH - 0.75, new Rotation2d(-0.5 * Math.PI));

  public static Transform2d LEFT_TRANSFORM = new Transform2d(-1.33, Units.inchesToMeters(6.5), new Rotation2d(Math.PI));
  public static Transform2d RIGHT_TRANSFORM = new Transform2d(-1.33, Units.inchesToMeters(-6.5), new Rotation2d(Math.PI));

  public static double RADIANS_PER_METER_EQUIVALENCE = Math.PI / 4.69;

  public static Transform2d L1_TRANSFORM = new Transform2d(0.2, 0, new Rotation2d());
  public static Transform2d TROUGH_TRANSFORM_LEFT = new Transform2d(0.2, 0, Rotation2d.fromDegrees(30));
  public static Transform2d TROUGH_TRANSFORM_RIGHT = new Transform2d(0.2, 0, Rotation2d.fromDegrees(-30));
  public static Transform2d L2_TRANSFORM = new Transform2d(0.4, 0, new Rotation2d());
  public static Transform2d L3_TRANSFORM = new Transform2d(1.25, 0, new Rotation2d());
  public static Transform2d ALGAE_TRANSFORM = new Transform2d(1.35, 0, new Rotation2d());

  public static Transform2d HOLDING_TOLERANCE_TRANSFORM = new Transform2d(DriveConstants.L1_LINEAR_TOLERANCE_METERS - 0.05, 0, new Rotation2d());

  public static Pose2d REEF_CENTER_BLUE = new Pose2d(4.5, aprilTagFieldLayout.getFieldWidth() / 2, new Rotation2d());

  public static Pose2d REEF_CENTER_RED = new Pose2d(aprilTagFieldLayout.getFieldLength() - 4.5, aprilTagFieldLayout.getFieldWidth() / 2, new Rotation2d());

  public static Translation2d HP_0 = new Translation2d(0, 0);
  public static Translation2d HP_1 = new Translation2d(0, FIELD_LENGTH);
  public static Translation2d HP_2 = new Translation2d(FIELD_WIDTH, 0);
  public static Translation2d HP_3 = new Translation2d(FIELD_WIDTH, FIELD_LENGTH);

  public static final double HP_ZONE = 3.5;

  // TODO: Check if Red Reef Positions are correct 
  public static HashMap<ReefPositions, Pose2d> reefPositionPoseRight = new HashMap<>() {{ 
    put(ReefPositions.ARB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.BRB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.CRB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.DRB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.ERB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.FRB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(RIGHT_TRANSFORM));
    put(ReefPositions.ARR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(RIGHT_TRANSFORM)); // 0
    put(ReefPositions.BRR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(RIGHT_TRANSFORM)); // 60
    put(ReefPositions.CRR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(RIGHT_TRANSFORM)); // 120
    put(ReefPositions.DRR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(RIGHT_TRANSFORM)); // 180
    put(ReefPositions.ERR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(RIGHT_TRANSFORM)); // 240
    put(ReefPositions.FRR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(RIGHT_TRANSFORM)); // 300
  }};

  public static HashMap<ReefPositions, Pose2d> reefPositionPoseLeft = new HashMap<>() {{
    put(ReefPositions.ALB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.BLB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.CLB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.DLB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.ELB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.FLB, REEF_CENTER_BLUE.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(LEFT_TRANSFORM));
    put(ReefPositions.ALR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(180))).transformBy(LEFT_TRANSFORM)); // 0
    put(ReefPositions.BLR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(240))).transformBy(LEFT_TRANSFORM)); // 60
    put(ReefPositions.CLR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(300))).transformBy(LEFT_TRANSFORM)); // 120
    put(ReefPositions.DLR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(0))).transformBy(LEFT_TRANSFORM)); // 180
    put(ReefPositions.ELR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(60))).transformBy(LEFT_TRANSFORM)); // 240
    put(ReefPositions.FLR, REEF_CENTER_RED.plus(GeomUtil.toTransform2d(Rotation2d.fromDegrees(120))).transformBy(LEFT_TRANSFORM)); // 300
  }};

  public static ReefPositions findClosestReefPoseLeft() {
    return findClosestReefPose(reefPositionPoseLeft);
  }

  public static ReefPositions findClosestReefPoseRight() {
    return findClosestReefPose(reefPositionPoseRight);
  }

  public static Pose2d findClosestProcessor() {
    Pose2d currentPose = Drive.getInstance().getPose();
    double blueDist = currentPose.getTranslation().getDistance(PROCESSOR_BLUE.getTranslation());
    double redDist = currentPose.getTranslation().getDistance(PROCESSOR_RED.getTranslation());

    if (blueDist < redDist) return PROCESSOR_BLUE;
    return PROCESSOR_RED;
  }

  private static ReefPositions findClosestReefPose(HashMap<ReefPositions, Pose2d> reefPositionPose) {
    Pose2d currentPose = Drive.getInstance().getPose();
    double smallestMagnitudeOfChange = 99999999999.0;

    ReefPositions closestReefPosition = ReefPositions.ALB;

    for (Map.Entry<ReefPositions, Pose2d> entry : reefPositionPose.entrySet()) {
      double currentDistance = Math.abs(currentPose.getTranslation().getDistance(entry.getValue().getTranslation()));
      double currentRotationRadians = Math.abs(currentPose.getRotation().minus(entry.getValue().getRotation()).getRadians());

      double currentMagnitudeOfChange = Math.hypot(currentDistance, (1 / RADIANS_PER_METER_EQUIVALENCE) * currentRotationRadians);

      if (currentMagnitudeOfChange < smallestMagnitudeOfChange) {
        smallestMagnitudeOfChange = currentMagnitudeOfChange;
        closestReefPosition = entry.getKey();
      }
    }

    return closestReefPosition;
  }

  public static Pose2d reefPositionToPose2d(ReefPositions reefPosition) {
    return reefPositionPoseLeft.getOrDefault(reefPosition, reefPositionPoseRight.get(reefPosition));
  }
}
