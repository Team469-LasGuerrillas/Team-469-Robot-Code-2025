package frc.lib.util.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.interfaces.motor.MotorConfigs;

import java.util.function.Supplier;

public class CTREUtil {
  public static final int MAX_RETRIES = 10;

  public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
    final int max_num_retries = 10;
    StatusCode statusCode = StatusCode.OK;
    for (int i = 0; i < max_num_retries; ++i) {
      statusCode = function.get();
      if (statusCode == StatusCode.OK) break;
    }
    if (statusCode != StatusCode.OK) {
      DriverStation.reportError(
          "Error calling " + function + " on ctre device id " + deviceId + ": " + statusCode, true);
    }
    return statusCode;
  }

  public static StatusCode applyFusedCancoderToTalon(MotorConfigs mConfig, TalonFX motor, CANcoder cancoder) {
    mConfig.fxConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    mConfig.fxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    mConfig.fxConfig.Feedback.SensorToMechanismRatio = mConfig.sensorToMechanismRatio;
    mConfig.fxConfig.Feedback.RotorToSensorRatio = mConfig.rotorToSensorRatio;

    return tryUntilOK(() -> motor.getConfigurator().refresh(mConfig.fxConfig), cancoder.getDeviceID());
  }

  public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
    return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
  }

  public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
    return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
  }

  public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
    return tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
  }

  public static StatusCode applyConfiguration(CANrange canRange, CANrangeConfiguration config) {
    return tryUntilOK(() -> canRange.getConfigurator().apply(config), canRange.getDeviceID());
  }

  public static StatusCode applyConfiguration(CANdi candi, CANdiConfiguration config) {
    return tryUntilOK(() -> candi.getConfigurator().apply(config), candi.getDeviceID());
  }

  public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
    return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
  }
}
