package frc.lib.drivecontrollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.Station;
import frc.lib.util.math.SignedPower;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class TeleopDriveController {

  private double controllerX = 0;
  private double controllerY = 0;
  private double controllerOmega = 0;
  private boolean robotRelative = false;

  public void acceptDriveInput(double x, double y, double omega, boolean robotRelative) {
    controllerX = x;
    controllerY = y;
    controllerOmega = omega;
    this.robotRelative = robotRelative;
  }

  public ChassisSpeeds update() {
    Translation2d linearVelocity = calcLinearVelocity(controllerX, controllerY);
    double omega = MathUtil.applyDeadband(controllerOmega, DriveConstants.STICK_DEADBAND);
    // omega = Math.copySign(omega * omega, omega);

    final double maxLinearVelocity = DriveConstants.TELEOP_MAX_LINEAR_VELOCITY;
    final double maxAngularVelocity = DriveConstants.TELEOP_MAX_ANGULAR_VELOCITY;

    if (robotRelative) {
      return new ChassisSpeeds(
          linearVelocity.getX() * maxLinearVelocity,
          linearVelocity.getY() * maxLinearVelocity,
          omega * maxAngularVelocity);
    } else {
      if (Station.isRed()) {
        linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
      }

      return ChassisSpeeds.fromFieldRelativeSpeeds(
          linearVelocity.getX() * maxLinearVelocity,
          linearVelocity.getY() * maxLinearVelocity,
          omega * maxAngularVelocity,
          Drive.getInstance().getRotation()
          );
    }
  }

  public static Translation2d calcLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.STICK_DEADBAND);

    Rotation2d linearDirection;
    if (linearMagnitude == 0) {
      linearDirection = new Rotation2d();
    } else {
      linearDirection = new Rotation2d(x, y);
    }

    // Square magnitude
    linearMagnitude = SignedPower.calculate(linearMagnitude, DriveConstants.STICK_EXPO);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    return linearVelocity;
  }
}
