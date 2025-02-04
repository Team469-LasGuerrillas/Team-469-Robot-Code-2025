package frc.lib.drivecontrollers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FieldRobotSpeedsConversion {
  public static ChassisSpeeds fieldToRobotSpeeds(ChassisSpeeds fieldSpeeds, Rotation2d robotRotation) {
    double x = fieldSpeeds.vxMetersPerSecond;
    double y = fieldSpeeds.vyMetersPerSecond;
    double theta = fieldSpeeds.omegaRadiansPerSecond;

    double drivingDistance = Math.hypot(x, y);
    double angle = Math.atan2(y, x) - robotRotation.getRadians();

    double realX = drivingDistance * Math.cos(angle);
    double realY = drivingDistance * Math.sin(angle);

    return new ChassisSpeeds(realX, realY, 0);
  }
}
