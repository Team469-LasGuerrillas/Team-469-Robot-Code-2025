package frc.lib.util.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class InterpolatorUtil {
  public static ChassisSpeeds chassisSpeeds(
      ChassisSpeeds base, ChassisSpeeds other, double interpolation) {
    interpolation = MathUtil.clamp(interpolation, 0, 1);

    ChassisSpeeds difference = other.minus(base);
    difference.times(interpolation);
    return base.plus(difference);
  }

  public static Twist2d twist2d(Twist2d base, Twist2d other, double interpolation) {
    interpolation = MathUtil.clamp(interpolation, 0, 1);

    return new Twist2d(
        MathUtil.interpolate(base.dx, other.dx, interpolation),
        MathUtil.interpolate(base.dy, other.dy, interpolation),
        MathUtil.interpolate(base.dtheta, other.dtheta, interpolation));
  }
}
