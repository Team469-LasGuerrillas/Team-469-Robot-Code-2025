package frc.lib.interfaces.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class MotorConfigs {
  public int canId;
  public CANBus canBus;
  public TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  // Ratio of rotor to units for this talon. rotor * by this ratio should
  // be the units.
  // <1 is reduction
  public double unitToRotorRatio = 1.0;
  public double sensorToMechanismRatio = 1.0;
  public double rotorToSensorRatio = 1.0;

  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

  public MotorConfigs withCanId(int canId) {
    this.canId = canId;
    return this;
  }

  public MotorConfigs withCanBus(CANBus canBus) {
    this.canBus = canBus;
    return this;
  }

  public MotorConfigs withFxConfig(TalonFXConfiguration fxConfig) {
    this.fxConfig = fxConfig;
    return this;
  }

  public MotorConfigs withUnitToRotorRatio(double unitToRotorRatio) {
    this.unitToRotorRatio = unitToRotorRatio;
    return this;
  }

  public MotorConfigs withSensorToMechanismRatio(double sensorToMechanismRatio) {
    this.sensorToMechanismRatio = sensorToMechanismRatio;
    return this;
  }

  public MotorConfigs withRotorToSensorRatio(double rotorToSensorRatio) {
    this.rotorToSensorRatio = rotorToSensorRatio;
    return this;
  }

  public MotorConfigs withMinPositionUnits(double kMinPositionUnits) {
    this.kMinPositionUnits = kMinPositionUnits;
    return this;
  }

  public MotorConfigs withMaxPositionUnits(double kMaxPositionUnits) {
    this.kMaxPositionUnits = kMaxPositionUnits;
    return this;
  }
}
