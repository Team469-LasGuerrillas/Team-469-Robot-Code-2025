package frc.lib.util.math;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.Drive;

public class TrigEstimator {
    private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static Pose3d getTrigBasedEstimatedPose(double distance3d, double tx, double ty, double timestamp, Pose3d robotToCam, int tagId) {
        Optional<Pose2d> timestampedRobotPose = Drive.getInstance().getTimestampedPose(timestamp);
        Rotation2d timestampedRobotRotation = timestampedRobotPose.get().getRotation();
        Pose2d tagPose2d = aprilTagFieldLayout.getTagPose(tagId).get().toPose2d();
        
        double distance2d = distance3d * Math.cos(robotToCam.getRotation().getY() + ty);
        
        Rotation2d camToTagRotation =
        timestampedRobotPose
            .get()
            .getRotation()
            .plus(robotToCam.toPose2d().getRotation().plus(Rotation2d.fromDegrees(tx)));
        
        Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(GeomUtil.toTransform2d(distance2d, 0.0))
            .getTranslation();
        
        Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation,
                timestampedRobotPose.get().getRotation().plus(robotToCam.toPose2d().getRotation()))
            .transformBy(new Transform2d(robotToCam.toPose2d(), Pose2d.kZero));
        
        robotPose = new Pose2d(robotPose.getTranslation(), timestampedRobotPose.get().getRotation());

        return new Pose3d(robotPose);

        
        // double distance2d = distance3d * Math.cos(Math.toRadians(ty) + robotToCam.getRotation().getY());
        // System.out.println("Distance 2d: " + distance2d + " Distance 3D: " + distance3d);

        // Rotation2d cameraToTagRotation = 
        //     timestampedRobotPose.get().getRotation()
        //         .plus(robotToCam.getRotation().toRotation2d())
        //         .plus(Rotation2d.fromDegrees(-tx));
        
        // Pose2d tagPose2d = aprilTagFieldLayout.getTagPose(tagId).get().toPose2d();

        // Translation2d fieldToCameraTranslation = 
        //     new Pose2d(tagPose2d.getTranslation(), cameraToTagRotation.plus(Rotation2d.kPi))
        //         .transformBy(GeomUtil.toTransform2d(distance2d, 0.0))
        //         .getTranslation();

        // Pose2d aprilTagSpacePose = 
        //     new Pose2d(distance2d, 0, new Rotation2d())
        //         .rotateBy(cameraToTagRotation.unaryMinus());
        
        // Pose2d realPose = tagPose2d.plus(GeomUtil.toTransform2d(aprilTagSpacePose)).plus(GeomUtil.toTransform2d(robotToCam.toPose2d()).inverse());

        // // Translation3d robotTranslation3d = new Translation3d(fieldToCameraTranslation.minus(robotToCam.getTranslation().toTranslation2d()));

        // // Pose2d robotPose =
        // //     new Pose2d(
        // //         fieldToCameraTranslation,
        // //         timestampedRobotRotation.plus(robotToCam.toPose2d().getRotation()))
        // //     .transformBy(new Transform2d(robotToCam.toPose2d(), Pose2d.kZero));
        
        // // robotPose = new Pose2d(robotPose.getTranslation(), timestampedRobotRotation);
        
        // // return new Pose3d(robotTranslation3d, new Rotation3d());
        // return new Pose3d(realPose);
        // return new Pose3d();
    }
}
