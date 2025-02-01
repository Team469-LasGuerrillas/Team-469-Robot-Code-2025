package frc.lib.util.math.estimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.util.math.odometry.OdometryType;
import frc.lib.util.math.odometry.VROdometry;

public class SequencingSwerveDrivePoseEstimator
    extends PoseSequencingEstimator<SwerveModulePosition[]> {

  public SequencingSwerveDrivePoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose3d questPoseRobotSpace,
      OdometryType odometryType) {
    super(
        kinematics,
        new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, new Pose2d()),
        new VROdometry(questPoseRobotSpace),
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.1, 0.1, 0.1),
        odometryType);
  }
}
