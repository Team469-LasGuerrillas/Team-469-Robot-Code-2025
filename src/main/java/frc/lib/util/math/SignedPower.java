package frc.lib.util.math;

public class SignedPower {
  public static double calculate(double input, double power) {
    boolean positive = true;
    double powerValue = Math.pow(input, power);

    if (input < 0) {
      positive = false;
    }

    if (positive) {
      if (powerValue < 0) {
        powerValue *= -1;
      }
    } else {
      if (powerValue > 0) {
        powerValue *= -1;
      }
    }

    return powerValue;
  }
}
