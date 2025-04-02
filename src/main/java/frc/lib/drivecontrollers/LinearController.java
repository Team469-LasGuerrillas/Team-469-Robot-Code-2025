package frc.lib.drivecontrollers;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.FieldLayout;
import frc.lib.util.Station;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class LinearController {
  
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final Supplier<Pose2d> goalPoseSupplier;

  double maxLinearVelocity = DriveConstants.TELEOP_MAX_LINEAR_VELOCITY;
  double maxLinearAcceleration = DriveConstants.MAX_LINEAR_ACCEL_FINE;

  ChassisSpeeds outputLinearSpeeds;

  public LinearController(Supplier<Pose2d> goalPoseSupplier, boolean runFineStep) {
    double pValue;
    double iValue;
    double dValue;
    double maxLinearAcceleration;

    if (runFineStep) {
      pValue = DriveConstants.LINEAR_P_FINE;
      iValue = DriveConstants.LINEAR_I_FINE;
      dValue = DriveConstants.LINEAR_D_FINE;
      maxLinearAcceleration = DriveConstants.MAX_LINEAR_ACCEL_FINE;
    }
    else {
      pValue = DriveConstants.LINEAR_P_GROSS;
      iValue = DriveConstants.LINEAR_I_GROSS;
      dValue = DriveConstants.LINEAR_D_GROSS;
      maxLinearAcceleration = DriveConstants.MAX_LINEAR_ACCEL_GROSS;
    }

    xController =
        new ProfiledPIDController(
            pValue,
            iValue,
            dValue,
            new TrapezoidProfile.Constraints(
              maxLinearVelocity,
              maxLinearAcceleration
            ),
            0.02);
    xController.setTolerance(DriveConstants.LINEAR_TOLERANCE_METERS);
    xController.setIZone(DriveConstants.I_ZONE_METERS);

    yController =
    new ProfiledPIDController(
        pValue,
        iValue,
        dValue,
        new TrapezoidProfile.Constraints(
          maxLinearVelocity,
          maxLinearAcceleration
        ),
        0.02);
    yController.setTolerance(DriveConstants.LINEAR_TOLERANCE_METERS);
    yController.setIZone(DriveConstants.I_ZONE_METERS);

    this.goalPoseSupplier = goalPoseSupplier;

    xController.reset(
      Drive.getInstance().getPose().getX(), 
      Drive.getInstance().getFieldVelocity().vxMetersPerSecond * DriveConstants.FIELD_VELOCITY_CORRECTION_FACTOR_MAGIC_NUMBER
    );

    yController.reset(
      Drive.getInstance().getPose().getY(), 
      Drive.getInstance().getFieldVelocity().vyMetersPerSecond * DriveConstants.FIELD_VELOCITY_CORRECTION_FACTOR_MAGIC_NUMBER
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
    if (Math.abs(xController.getPositionError()) > DriveConstants.LINEAR_TOLERANCE_METERS 
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

  public double getError() {
    double xError = xController.getPositionError(); 
    double yError = yController.getPositionError();
    return Math.hypot(xError, yError);
  }

  public double getTargetDistanceFromReefCenter() {
    double xError;
    if (Station.isRed()) xError = Math.abs(FieldLayout.REEF_CENTER_RED.getX() - xController.getGoal().position);
    else xError = Math.abs(FieldLayout.REEF_CENTER_BLUE.getX() - xController.getGoal().position);

    double yError;
    if (Station.isRed()) yError = Math.abs(FieldLayout.REEF_CENTER_RED.getY() - yController.getGoal().position);
    else yError = Math.abs(FieldLayout.REEF_CENTER_BLUE.getY() - yController.getGoal().position);

    return Math.hypot(xError, yError);
  }
}
