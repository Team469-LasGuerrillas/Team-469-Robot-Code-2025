package frc.lib.util;

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
}
