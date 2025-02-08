package frc.lib.util.math;

public class ToleranceUtil {
  public static boolean epsilonEquals(double current, double goal, double tolerance) {
    double difference = Math.abs(current - goal);
    tolerance = Math.abs(tolerance);

    if (difference <= tolerance) {
      return true;
    } else {
      return false;
    }
  }

  public static boolean epsilonEqualsRadialDegrees(double current, double goal, double tolerance) {
    double difference = Math.abs(current - goal);
    double otherDifference  = Math.abs((current + 360) - goal);
    tolerance = Math.abs(tolerance);

    if (difference <= tolerance) {
      return true;
    } else if (otherDifference <= tolerance) {
      return true;
    } else {
      return false;
    }
  }
}
