package frc.lib.interfaces;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AndrewShi extends MonkeyAhhPoseEstimator<SwerveModulePosition[]> {
  
  public AndrewShi(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters) {
    super(
        kinematics,
        new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters),
        new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters),
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.1, 0.1, 0.1));
  }
}
