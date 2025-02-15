package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;

public class AlgaeEndEffectorConstants {
    public static final double ALGAE_INTAKE_IN_VOLTAGE = 12;
    public static final double ALGAE_INTAKE_OUT_VOLAGE = -12;

    public static final double ALGAE_WRIST_DEFAULT = 0;
    public static final double ALGAE_WRIST_PROCESSOR = 0;
    public static final double ALGAE_WRIST_BARGE = 0;
    public static final double ALGAE_WRIST_GROUND = 0;
    public static final double ALGAE_WRIST_REEF = 0;

    /* ALGAE INTAKE MOTOR */
    private static TalonFXConfiguration algaeIntakeMotorFxConfig = 
    new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20));
    
    private static MotorConfigs algaeIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(algaeIntakeMotorFxConfig);
    
    public static MotorIOTalonFX algaeIntakeMotor = new MotorIOTalonFX(algaeIntakeMotorConfigs);

    /* ALGAE WIRST MOTOR */
    
    private static TalonFXConfiguration algaeWristMotorFxConfig = 
    new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(23));

    private static MotorConfigs algaeWristMotorConfigs = new MotorConfigs()    
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(algaeWristMotorFxConfig);

    public static MotorIOTalonFX algaeWristMotor = new MotorIOTalonFX(algaeWristMotorConfigs);
}