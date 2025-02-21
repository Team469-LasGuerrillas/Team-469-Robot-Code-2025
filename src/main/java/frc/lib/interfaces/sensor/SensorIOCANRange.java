package frc.lib.interfaces.sensor;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import frc.lib.util.hardware.CTREUtil;
import frc.robot.commandfactories.AlgaeEndEffectorCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.SensorConstants;

public class SensorIOCANRange implements SensorIO {
  protected CANrange canRange;

  public SensorIOCANRange(CANrangeConfiguration canRangeConfiguration, int canId) {
    canRange = new CANrange(canId, TunerConstants.kCANBus);

    CTREUtil.applyConfiguration(canRange, canRangeConfiguration);
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.distance = canRange.getDistance().getValueAsDouble();
  }
}
