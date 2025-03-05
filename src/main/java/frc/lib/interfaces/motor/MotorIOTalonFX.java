package frc.lib.interfaces.motor;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.hardware.CTREUtil;

public class MotorIOTalonFX implements MotorIO {
  protected TalonFX talon;
  protected CANcoder cancoder;

  protected final MotorConfigs mConfig;

  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
  private final MotionMagicVelocityVoltage motionMagicVelocityControl =
      new MotionMagicVelocityVoltage(0.0);
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
  private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentStatorSignal;
  public BaseStatusSignal[] signals;

  public MotorIOTalonFX(MotorConfigs config, MotorIOTalonFX... followerMotors) {
    this.mConfig = config;
    talon = new TalonFX(config.canId, config.canBus);

    CTREUtil.applyConfiguration(talon, config.fxConfig);

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();

    signals =
        new BaseStatusSignal[] {
          positionSignal, velocitySignal, voltageSignal, currentStatorSignal
        };

    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), talon.getDeviceID());
    CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());

    for (int i = 0; i < followerMotors.length; i++) {
      followerMotors[i].talon.setControl(new StrictFollower(talon.getDeviceID()));
    }
  }

  public MotorIOTalonFX(MotorConfigs mConfig, CancoderConfigs ccConfig) {
    this(mConfig);

    cancoder = new CANcoder(ccConfig.canId, ccConfig.canBus);
    CTREUtil.applyConfiguration(cancoder, ccConfig.ccConfig);

    CTREUtil.applyFusedCancoderToTalon(mConfig, talon, cancoder);
    
    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), cancoder.getDeviceID());
    CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), cancoder.getDeviceID());
  }

  private double rotorToUnits(double rotor) {
    return rotor * mConfig.unitToRotorRatio;
  }

  private double clampPosition(double units) {
    return unitsToRotor(MathUtil.clamp(units, mConfig.kMinPositionUnits, mConfig.kMaxPositionUnits));
  }

  public double unitsToRotor(double units) {
    return units / mConfig.unitToRotorRatio;
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    inputs.unitPosition = rotorToUnits(positionSignal.getValueAsDouble());
    inputs.velocityUnitsPerSecond = rotorToUnits(velocitySignal.getValueAsDouble());
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();

    inputs.canbusId = mConfig.canId;

    inputs.cancoderPos = cancoder == null ? 0.0 : cancoder.getPosition().getValueAsDouble();
  }

  @Override
  public void setOpenLoopVoltage(DoubleSupplier volts) {
    talon.setVoltage(volts.getAsDouble());
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
    mConfig.fxConfig.MotorOutput.NeutralMode = mode;
    CTREUtil.applyConfiguration(talon, mConfig.fxConfig);
  }

  @Override
  public void setCurrentPosition(double units) {
    talon.setPosition(units / mConfig.unitToRotorRatio);
  }

  @Override
  public void setCurrentPositionAsZero() {
    setCurrentPosition(0.0);
  }

  @Override
  public void setSoftLimits(boolean fwd, boolean rev, double fwdLimit, double revLimit) {
    mConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = fwd;
    mConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = rev;
    mConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = fwdLimit;
    mConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = revLimit;
    CTREUtil.applyConfiguration(talon, mConfig.fxConfig);
  }
}
