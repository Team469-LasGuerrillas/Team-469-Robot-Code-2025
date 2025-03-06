package frc.lib.interfaces.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  @AutoLog
  class MotorIOInputs {
    public boolean connected = false;

    public double velocityUnitsPerSecond = 0.0;
    public double unitPosition = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;

    public double positionError = 0.0;

    public double temperatureCelcius = 0.0;
    public int canbusId;

    public double cancoderPos = 0.0;
  }

  public default void updateInputs(MotorIOInputs inputs) {}

  // Open-Loop Voltage
  public default void setOpenLoopVoltage(DoubleSupplier volts) {}

  // Position
  public default void setPositionSetpoint(double units, double feedForward) {}

  // MotionMagic Position
  public default void setMagicalPositionSetpoint(double units, double feedForward) {}

  // Velocity
  public default void setVelocitySetpoint(double unitsPerSecond) {}

  // Motion Magic Velocity
  public default void setMagicalVelocitySetpoint(double unitsPerSecond) {}

  // Idle Mode
  public default void setIdleMode(NeutralModeValue mode) {}

  // Reset Position To Zero
  public default void setCurrentPositionAsZero() {}

  // Redefine Current Position
  public default void setCurrentPosition(double units) {}

  // Soft Limits
  public default void setSoftLimits(boolean fwd, boolean rev, double fwdLimit, double revLimit) {}
}
