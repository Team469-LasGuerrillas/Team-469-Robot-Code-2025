package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.lib.interfaces.motor.MotorConfigs;

public class ClimbConstants {

    /*CLIMB MOTOR */
    public static final double SLOW_RETRACT = -2;
    public static final double FAST_RETRACT = 12;
    public static final double DEPLOY = 1;

    private static TalonFXConfiguration climbMotorFxConfig = 
    new TalonFXConfiguration()
    .withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
    )
    .withCurrentLimits(
        new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(100));

    private static MotorConfigs climbMotorConfigs = new MotorConfigs()
        .withCanId(10)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(climbMotorFxConfig);

    public static MotorIOTalonFX climbMotor = new MotorIOTalonFX(climbMotorConfigs);
}