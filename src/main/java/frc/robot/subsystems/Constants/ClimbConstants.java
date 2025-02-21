package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;

public class ClimbConstants {

    /*CLIMB MOTOR */
    public static final double CLIMB_EXTEND = 12;
    public static final double CLIMB_RETRACT = -12;

    private static TalonFXConfiguration climbMotorFxConfig = 
    new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(21));

    private static MotorConfigs climbMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(climbMotorFxConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);

    public static MotorIOTalonFX climbMotor = new MotorIOTalonFX(climbMotorConfigs);
}