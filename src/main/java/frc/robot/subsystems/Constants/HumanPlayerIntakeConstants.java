package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;

public class HumanPlayerIntakeConstants {
    private final double hpIntakeMotorStatorCurrentLimit = 20;

    /* HUMAN PLAYER INTAKE MOTOR */
    TalonFXConfiguration hpIntakeTalonFXConfig = 
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(hpIntakeMotorStatorCurrentLimit));

    MotorConfigs hpIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(hpIntakeTalonFXConfig);

    MotorIOTalonFX hpIntakeMotor = new MotorIOTalonFX(hpIntakeMotorConfigs);
}