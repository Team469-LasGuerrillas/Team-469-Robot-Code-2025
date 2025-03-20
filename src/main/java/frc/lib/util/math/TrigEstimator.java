package frc.lib.util.math;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;

public class TrigEstimator {
    private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static Pose3d getTrigBasedEstimatedPose(double distance3d, double tx, double ty, double timestamp, Pose3d cameraPose, int tagId) {
        Optional<Pose2d> timestampedRobotPose = Drive.getInstance().getTimestampedPose(timestamp);
        Rotation2d timestampedRobotRotation = timestampedRobotPose.get().getRotation();
        Pose2d tagPose2d = aprilTagFieldLayout.getTagPose(tagId).get().toPose2d();
        
        Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, Units.degreesToRadians(ty), Units.degreesToRadians(tx)))
            .transformBy(
                new Transform3d(new Translation3d(distance3d, 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, -cameraPose.getRotation().getY(), 0))
            .toTranslation2d();
            
        Rotation2d camToTagRotation =
            timestampedRobotRotation.plus(
                cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));

        Translation2d fieldToCameraTranslation =
            new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
                .getTranslation();

        Pose2d robotPose =
            new Pose2d(
                    fieldToCameraTranslation, timestampedRobotRotation.plus(cameraPose.toPose2d().getRotation()))
                .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));

        // Use gyro angle at time for robot rotation
        robotPose = new Pose2d(robotPose.getTranslation(), timestampedRobotRotation);
        return new Pose3d(robotPose);
    }
}
