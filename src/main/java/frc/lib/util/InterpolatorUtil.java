package frc.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class InterpolatorUtil {
  public static ChassisSpeeds chassisSpeeds(
      ChassisSpeeds base, ChassisSpeeds other, double interpolation) {
    ChassisSpeeds difference = other.minus(base);
    difference.times(interpolation);
    return base.plus(difference);
  }
}
