package frc.lib.drivecontrollers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

import java.util.function.Supplier;

public class HeadingController {

  private final ProfiledPIDController controller;
  private final Supplier<Rotation2d> goalHeadingSupplier;

  public HeadingController(Supplier<Rotation2d> goalHeadingSupplier) {
    controller =
        new ProfiledPIDController(
            DriveConstants.HEADING_P,
            DriveConstants.HEADING_I,
            DriveConstants.HEADING_D,
            new TrapezoidProfile.Constraints(0.0, 0.0),
            0.02);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(Units.degreesToRadians(0));
    this.goalHeadingSupplier = goalHeadingSupplier;

    controller.reset(
        Drive.getInstance().getRotation().getRadians(), Drive.getInstance().fieldVelocity().dtheta);
  }

  public double update() {
    double maxAngularAcceleration = DriveConstants.MAX_ANGULAR_ACCEL;
    double maxAngularVelocity = DriveConstants.TELEOP_MAX_LINEAR_VELOCITY;

    controller.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));

    double output =
        controller.calculate(
            Drive.getInstance().getRotation().getRadians(), goalHeadingSupplier.get().getRadians());

    // Logger.recordOutput("Drive/HeadingController/HeadingError", controller.getPositionError());
    return output;
  }

  public boolean atGoal() {
    return ToleranceUtil.epsilonEquals(
        controller.getSetpoint().position,
        controller.getGoal().position,
        Units.degreesToRadians(DriveConstants.HEADING_TOLERANCE_DEGREES));
  }
}
