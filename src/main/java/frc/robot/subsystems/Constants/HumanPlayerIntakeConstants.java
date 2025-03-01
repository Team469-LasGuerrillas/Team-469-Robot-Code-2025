package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.interfaces.sensor.SensorIOBeamBreak;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.lib.interfaces.motor.MotorConfigs;

public class HumanPlayerIntakeConstants {

    /* SENSOR BEAM BREAKER */
    SensorIOBeamBreak beamBreak = new SensorIOBeamBreak(new CANdiConfiguration(), 0);

    /* HUMAN PLAYER INTAKE MOTOR */
    public static final double HP_INTAKE_IN = 12;
    public static final double HP_INTAKE_OUT = -12;
    public static final double HP_INTAKE_DEFAULT_VOLTAGE = 0;
    
    private static TalonFXConfiguration hpIntakeTalonFXConfig = 
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(20));

    private static MotorConfigs hpIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(hpIntakeTalonFXConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);

    public static MotorIOTalonFX hpIntakeMotor = new MotorIOTalonFX(hpIntakeMotorConfigs);
}