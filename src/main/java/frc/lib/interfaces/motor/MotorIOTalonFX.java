package frc.lib.interfaces.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.hardware.CTREUtil;

public class MotorIOTalonFX implements MotorIO {
  protected final TalonFX talon;
  protected final MotorConfigs config;

  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
  private final MotionMagicVelocityVoltage motionMagicVelocityControl =
      new MotionMagicVelocityVoltage(0.0);
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
  private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;
  public BaseStatusSignal[] signals;

  public MotorIOTalonFX(MotorConfigs config) {
    this.config = config;
    talon = new TalonFX(config.canId, config.canBus);

    CTREUtil.applyConfiguration(talon, this.config.fxConfig);

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();

    signals =
        new BaseStatusSignal[] {
          positionSignal, velocitySignal, voltageSignal, currentStatorSignal, currentSupplySignal
        };

    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), talon.getDeviceID());
    CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());
  }

  private double rotorToUnits(double rotor) {
    return rotor * config.unitToRotorRatio;
  }

  private double clampPosition(double units) {
    return unitsToRotor(MathUtil.clamp(units, config.kMinPositionUnits, config.kMaxPositionUnits));
  }

  public double unitsToRotor(double units) {
    return units / config.unitToRotorRatio;
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    inputs.unitPosition = rotorToUnits(positionSignal.getValueAsDouble());
    inputs.velocityUnitsPerSecond = rotorToUnits(velocitySignal.getValueAsDouble());
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();

    inputs.canbusId = config.canId;
  }

  @Override
  public void setOpenLoopVoltage(double volts) {
    talon.setVoltage(volts);
  }

  @Override
  public void setPositionSetpoint(double units, double feedForward) {
    talon.setControl(
        positionVoltageControl.withPosition(clampPosition(units)).withFeedForward(feedForward));
  }

  @Override
  public void setMagicalPositionSetpoint(double units, double feedForward) {
    talon.setControl(
        motionMagicPositionControl.withPosition(clampPosition(units)).withFeedForward(feedForward));
  }

  @Override
  public void setVelocitySetpoint(double unitsPerSecond) {
    talon.setControl(velocityVoltageControl.withVelocity(clampPosition(unitsPerSecond)));
  }

  @Override
  public void setMagicalVelocitySetpoint(double unitsPerSecond) {
    talon.setControl(motionMagicVelocityControl.withVelocity(clampPosition(unitsPerSecond)));
  }

  @Override
  public void setIdleMode(NeutralModeValue mode) {
    config.fxConfig.MotorOutput.NeutralMode = mode;
    CTREUtil.applyConfiguration(talon, config.fxConfig);
  }

  @Override
  public void setCurrentPosition(double units) {
    talon.setPosition(units / config.unitToRotorRatio);
  }

  @Override
  public void setCurrentPositionAsZero() {
    setCurrentPosition(0.0);
  }

  @Override
  public void setSoftLimits(boolean fwd, boolean rev, double fwdLimit, double revLimit) {
    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = fwd;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = rev;
    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = fwdLimit;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = revLimit;
    CTREUtil.applyConfiguration(talon, config.fxConfig);
  }
}
