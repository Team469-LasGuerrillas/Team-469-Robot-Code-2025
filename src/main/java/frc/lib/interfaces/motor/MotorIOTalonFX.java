package frc.lib.interfaces.motor;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import frc.robot.util.Music;

public class MotorIOTalonFX implements MotorIO {
  protected TalonFX talon;
  protected CANcoder cancoder;

  protected final MotorConfigs mConfig;

  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);
  private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityControl = new MotionMagicVelocityTorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC motionMagicPositionControl = new MotionMagicTorqueCurrentFOC(0.0);
  private final DynamicMotionMagicTorqueCurrentFOC dynamicMotionMagicPositionControl = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0);

  private StatusSignal<Angle> positionSignal;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Current> currentStatorSignal;
  
  public BaseStatusSignal[] signals;

  public int slot = 0;
  public double velocity = 99999;

  public MotorIOTalonFX(MotorConfigs config, MotorIOTalonFX... followerMotors) {
    this.mConfig = config;
    config.fxConfig.Audio.AllowMusicDurDisable = true;

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

    Music.addInstrument(talon);

    for (int i = 0; i < followerMotors.length; i++) {
      followerMotors[i].talon.setControl(new StrictFollower(talon.getDeviceID()));
      Music.addInstrument(followerMotors[i].talon);
    }
  }

  public MotorIOTalonFX(MotorConfigs mConfig, CancoderConfigs ccConfig) {
    this.mConfig = mConfig;
    mConfig.fxConfig.Audio.AllowMusicDurDisable = true;

    talon = new TalonFX(mConfig.canId, mConfig.canBus);
    cancoder = new CANcoder(ccConfig.canId, ccConfig.canBus);

    positionSignal = cancoder.getPosition();
    velocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();

    signals =
        new BaseStatusSignal[] {
          positionSignal, velocitySignal, voltageSignal, currentStatorSignal
        };

    CTREUtil.applyConfiguration(cancoder, ccConfig.ccConfig);
    CTREUtil.applyFusedCancoderToTalon(mConfig, talon, cancoder);

    CTREUtil.tryUntilOK(
      () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), talon.getDeviceID());
    CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());

    Music.addInstrument(talon);
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

    inputs.velocityUnitsPerSecond = rotorToUnits(velocitySignal.getValueAsDouble());
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();

    inputs.unitPosition = rotorToUnits(positionSignal.getValueAsDouble());
    inputs.positionError = rotorToUnits(talon.getClosedLoopError().getValueAsDouble());
    inputs.targetPosition = rotorToUnits(talon.getClosedLoopReference().getValueAsDouble());

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
        positionControl.withPosition(clampPosition(units)).withFeedForward(feedForward).withSlot(slot));
  }

  @Override
  public void setMagicalPositionSetpoint(double units, double feedForward) {
    talon.setControl(
        motionMagicPositionControl.withPosition(clampPosition(units)).withFeedForward(feedForward).withSlot(slot));
  }

  @Override
  public void setDynamicMagicalPositionSetpoint(double units, double feedForward, double velocityUnits, double accelerationUnits, double jerkUnits) {
    dynamicMotionMagicPositionControl.Velocity = velocityUnits;
    dynamicMotionMagicPositionControl.Acceleration = accelerationUnits;
    dynamicMotionMagicPositionControl.Jerk = jerkUnits;
    
    talon.setControl(
      dynamicMotionMagicPositionControl.withPosition(clampPosition(units)).withFeedForward(feedForward).withSlot(slot)
    );
  }

  @Override
  public void setVelocitySetpoint(double unitsPerSecond) {
    talon.setControl(velocityControl.withVelocity(clampPosition(unitsPerSecond)).withSlot(slot));
  }

  @Override
  public void setMagicalVelocitySetpoint(double unitsPerSecond) {
    talon.setControl(motionMagicVelocityControl.withVelocity(clampPosition(unitsPerSecond)).withSlot(slot));
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

  @Override
  public void setSlot(int slot) {
    this.slot = slot;
  }
}
