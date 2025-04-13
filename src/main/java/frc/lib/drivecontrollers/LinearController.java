package frc.lib.drivecontrollers;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.FieldLayout;
import frc.lib.util.Station;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class LinearController {
  
  private final PIDController xController;
  private final PIDController yController;
  private final Supplier<Pose2d> goalPoseSupplier;

  double maxLinearVelocity = DriveConstants.AUTO_VELOCITY;
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
        new PIDController(
            pValue,
            iValue,
            dValue,
            0.02);
    xController.setTolerance(DriveConstants.LINEAR_TOLERANCE_METERS);
    xController.setIZone(DriveConstants.I_ZONE_METERS);

    yController =
    new PIDController(
        pValue,
        iValue,
        dValue,
        0.02);
    yController.setTolerance(DriveConstants.LINEAR_TOLERANCE_METERS);
    yController.setIZone(DriveConstants.I_ZONE_METERS);

    this.goalPoseSupplier = goalPoseSupplier;
    double initialXVel = Drive.getInstance().getFieldVelocity().vxMetersPerSecond * DriveConstants.FIELD_VELOCITY_CORRECTION_FACTOR_MAGIC_NUMBER;
    double initialYVel = Drive.getInstance().getFieldVelocity().vyMetersPerSecond * DriveConstants.FIELD_VELOCITY_CORRECTION_FACTOR_MAGIC_NUMBER;

    System.out.println("Initial Velocity X: " + initialXVel + ". Initial Velocity Y: " + initialYVel);
    Logger.recordOutput("linearController/linearControllerInitialVelocity",new ChassisSpeeds(initialXVel, initialYVel, 0));

    // xController.reset(
    //   Drive.getInstance().getPose().getX(), 
    //   Drive.getInstance().getFieldVelocity().vxMetersPerSecond * DriveConstants.FIELD_VELOCITY_CORRECTION_FACTOR_MAGIC_NUMBER
    // );

    // yController.reset(
    //   Drive.getInstance().getPose().getY(), 
    //   Drive.getInstance().getFieldVelocity().vyMetersPerSecond * DriveConstants.FIELD_VELOCITY_CORRECTION_FACTOR_MAGIC_NUMBER
    // );
  }

  public ChassisSpeeds update() {
    double xOutput =
      xController.calculate(
        Drive.getInstance().getPose().getX(),
        goalPoseSupplier.get().getX());

    double yOutput =
      yController.calculate(
        Drive.getInstance().getPose().getY(), goalPoseSupplier.get().getY());
    
    ChassisSpeeds outputLinearSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, 0, Drive.getInstance().getRotation());
    Logger.recordOutput("linearController/linearControllerVelocity", outputLinearSpeeds);
    if (Math.abs(xController.getError()) > DriveConstants.LINEAR_TOLERANCE_METERS 
        || Math.abs(yController.getError()) > DriveConstants.LINEAR_TOLERANCE_METERS) {
          double netVelocity = Math.hypot(xOutput, yOutput);

          Logger.recordOutput("linearController/linearControllerVelocity", netVelocity);
          return outputLinearSpeeds;
    }
    else {
      return new ChassisSpeeds();
    }
  }

  public boolean atXGoal() {
    return Math.abs(xController.getError()) <
        DriveConstants.LINEAR_TOLERANCE_METERS;
  }

  public boolean atYGoal() {
    return Math.abs(yController.getError()) <
        DriveConstants.LINEAR_TOLERANCE_METERS;
  }

  public Pose2d getTargetPose() {
    return goalPoseSupplier.get();
  }

  public double getError() {
    double xError = xController.getError(); 
    double yError = yController.getError();
    return Math.hypot(xError, yError);
  }

  public double getTargetDistanceFromReefCenter() {
    double xError;
    if (Station.isRed()) xError = Math.abs(FieldLayout.REEF_CENTER_RED.getX() - goalPoseSupplier.get().getX());
    else xError = Math.abs(FieldLayout.REEF_CENTER_BLUE.getX() - goalPoseSupplier.get().getX());

    double yError;
    if (Station.isRed()) yError = Math.abs(FieldLayout.REEF_CENTER_RED.getY() - goalPoseSupplier.get().getY());
    else yError = Math.abs(FieldLayout.REEF_CENTER_BLUE.getY() - goalPoseSupplier.get().getY());

    return Math.hypot(xError, yError);
  }
}
