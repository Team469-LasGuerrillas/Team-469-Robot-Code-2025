package frc.lib.interfaces.sensor;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
  
  @AutoLog
  class SensorIOInputs {
    public double distance = 1;
    public boolean isCut = false;
  }

  public default void updateInputs(SensorIOInputs inputs) {}
}