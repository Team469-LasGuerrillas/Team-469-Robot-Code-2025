package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;

public class HumanPlayerIntakeConstants {
    /* HUMAN PLAYER INTAKE MOTOR */
    public static final double HP_INTAKE_IN = 12;
    public static final double HP_INTAKE_OUT = -12;
    public static final double HP_INTAKE_DEFAULT = 0;
    
    private static TalonFXConfiguration hpIntakeTalonFXConfig = 
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(20));

    private static MotorConfigs hpIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(hpIntakeTalonFXConfig);

    public static MotorIOTalonFX hpIntakeMotor = new MotorIOTalonFX(hpIntakeMotorConfigs);
}