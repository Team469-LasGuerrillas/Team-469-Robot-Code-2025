package frc.lib.interfaces.sensor;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1StateValue;
import frc.lib.util.hardware.CTREUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.constants.SensorConstants;
public class SensorIOBeamBreak implements SensorIO {
  protected CANdi candi;

  public SensorIOBeamBreak(CANdiConfiguration candiConfiguration, int canId) {
    candi = new CANdi(canId, TunerConstants.kCANBus);

    CTREUtil.applyConfiguration(candi, candiConfiguration);
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    double stateValue = candi.getS1State().getValueAsDouble();
    if (stateValue < SensorConstants.BEAM_BREAKER_CONNECTED_VALUE) {
      inputs.isCut = true;
    } else {
      inputs.isCut = false;
    }
  }
}