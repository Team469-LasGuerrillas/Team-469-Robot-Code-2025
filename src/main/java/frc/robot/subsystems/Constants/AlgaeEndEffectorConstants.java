package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;

public class AlgaeEndEffectorConstants {

    /* ALGAE INTAKE MOTOR */
    public static final double ALGAE_INTAKE_IN_VOLTAGE = 12;
    public static final double ALGAE_INTAKE_OUT_VOLTAGE = -12;
    public static final double ALGAE_INTAKE_BARGE_OUT_VOLTAGE = -24;
    public static final double ALGAE_INTAKE_DEFAULT_VOLTAGE = 0;

    private static TalonFXConfiguration algaeIntakeMotorFxConfig = 
    new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20));
    
    private static MotorConfigs algaeIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(algaeIntakeMotorFxConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);
    
    public static MotorIOTalonFX algaeIntakeMotor = new MotorIOTalonFX(algaeIntakeMotorConfigs);

    /* ALGAE WIRST MOTOR */
    public static final double ALGAE_WRIST_DEFAULT = 0;
    public static final double ALGAE_WRIST_PROCESSOR = 0;
    public static final double ALGAE_WRIST_BARGE = 0;
    public static final double ALGAE_WRIST_GROUND = 0;
    public static final double ALGAE_WRIST_REEF = 0;

    private static TalonFXConfiguration algaeWristMotorFxConfig = 
    new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(23))
        .withSlot0(
            new Slot0Configs()
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0))
        .withMotionMagic(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(80)
            .withMotionMagicAcceleration(90));

    private static MotorConfigs algaeWristMotorConfigs = new MotorConfigs()    
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(algaeWristMotorFxConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);

    public static MotorIOTalonFX algaeWristMotor = new MotorIOTalonFX(algaeWristMotorConfigs);
}