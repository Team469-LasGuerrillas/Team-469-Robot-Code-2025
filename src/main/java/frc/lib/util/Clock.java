package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class Clock {
  public static double time() {
    return Timer.getFPGATimestamp();
  }
}
