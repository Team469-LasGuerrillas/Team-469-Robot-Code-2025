package frc.lib.interfaces.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class MotorConfigs {
  public int canId;
  public String canBus;
  public final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  // Ratio of rotor to units for this talon. rotor * by this ratio should
  // be the units.
  // <1 is reduction
  public double unitToRotorRatio = 1.0;
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;
}
