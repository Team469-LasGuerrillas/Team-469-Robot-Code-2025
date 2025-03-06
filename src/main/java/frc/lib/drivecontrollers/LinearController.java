package frc.lib.drivecontrollers;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class LinearController {
  
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final Supplier<Pose2d> goalPoseSupplier;

  double maxLinearVelocity = DriveConstants.TELEOP_MAX_LINEAR_VELOCITY;
  double maxLinearAcceleration = DriveConstants.MAX_LINEAR_ACCEL;

  ChassisSpeeds outputLinearSpeeds;

  public LinearController(Supplier<Pose2d> goalPoseSupplier) {
    xController =
        new ProfiledPIDController(
            DriveConstants.LINEAR_P,
            DriveConstants.LINEAR_I,
            DriveConstants.LINEAR_D,
            new TrapezoidProfile.Constraints(
              maxLinearVelocity,
              maxLinearAcceleration
            ),
            0.02);
    xController.setTolerance(DriveConstants.LINEAR_TOLERANCE_METERS);

    yController =
    new ProfiledPIDController(
        DriveConstants.LINEAR_P,
        DriveConstants.LINEAR_I,
        DriveConstants.LINEAR_D,
        new TrapezoidProfile.Constraints(
          maxLinearVelocity,
          maxLinearAcceleration
        ),
        0.02);
    yController.setTolerance(DriveConstants.LINEAR_TOLERANCE_METERS);

    this.goalPoseSupplier = goalPoseSupplier;

    xController.reset(
      Drive.getInstance().getPose().getX(), 
      Drive.getInstance().fieldVelocity().dx
    );

    yController.reset(
      Drive.getInstance().getPose().getY(), 
      Drive.getInstance().fieldVelocity().dy
    );
  }

  public ChassisSpeeds update() {
    double xOutput =
      xController.calculate(
        Drive.getInstance().getPose().getX(),
        goalPoseSupplier.get().getX());

    double yOutput =
      yController.calculate(
        Drive.getInstance().getPose().getY(), goalPoseSupplier.get().getY());
    
    ChassisSpeeds outputLinearSpeeds = new ChassisSpeeds(xOutput, yOutput, 0);
    if (xController.getPositionError() > DriveConstants.LINEAR_TOLERANCE_METERS 
        || Math.abs(yController.getPositionError()) > DriveConstants.LINEAR_TOLERANCE_METERS) {
          return outputLinearSpeeds;
    }
    else {
      return new ChassisSpeeds();
    }
  }

  public boolean atXGoal() {
    return ToleranceUtil.epsilonEquals(
        xController.getSetpoint().position,
        xController.getGoal().position,
        Units.degreesToRadians(DriveConstants.LINEAR_TOLERANCE_METERS));
  }

  public boolean atYGoal() {
    return ToleranceUtil.epsilonEquals(
        yController.getSetpoint().position,
        yController.getGoal().position,
        Units.degreesToRadians(DriveConstants.LINEAR_TOLERANCE_METERS));
  }

  public Pose2d getTargetPose() {
    return goalPoseSupplier.get();
  }
}
