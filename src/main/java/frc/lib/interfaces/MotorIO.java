package frc.lib.interfaces;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  @AutoLog
  public static class MotorIOInputs {
    public double velocityUnitsPerSecond = 0.0;
    public double unitPosition = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;

    public double temperatureCelcius = 0.0;
    public int canbusId;
  }

  void readInputs(MotorIOInputs inputs);

  void setOpenLoopVoltage(double volts);

  void setPositionSetpoint(double units);

  void setMagicalPositionSetpoint(double units);

  void setVelocitySetpoint(double unitsPerSecond);

  void setMagicalVelocitySetpoint(double unitsPerSecond);

  void setFeedForwardVoltage(double volts);

  void setIdleMode();

  void setZeroPosition();

  void setPosition(double units);

  void setSoftLimits(boolean forward, boolean reverse);
}
