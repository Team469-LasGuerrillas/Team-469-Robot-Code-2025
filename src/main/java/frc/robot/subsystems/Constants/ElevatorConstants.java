package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;

public class ElevatorConstants {

    public static final double GROUND_TO_CORAL_REST_POS_INCHES = 4;
    public static final double CORAL_STAGE_HEIGHT_FACTOR = 3.5;
    public static final double ALGAE_RELATIVE_TO_CORAL_HEIGHT = 6;
    public static final double GROUND_TO_ALGAE_REST_POS_INCHES = 3;
    public static final double CORAL_CARRIAGE_TO_FIRST_STAGE_HEIGHT = 4;
    public static final double ALGAE_HEIGHT_IN_INCHES_FROM_FIRST_STAGE =5;
    public static final double ELEVATOR_FROM_GROUND = 3487;
    public static final double CORAL_CARRIAGE_TO_ALGAE_ZERO = 469;
    public static final double ALGAE_CARRIAGE_HEIGHT = 2;
    public static final double ALGAE_STAGE_HEIGHT_FACTOR = 765;

    /* CORAL ELEVATOR MOTOR */
    public static final double CORAL_STAGE_UP = 0;
    public static final double CORAL_STAGE_DOWN = 0;

    private static TalonFXConfiguration coralElevatorTalonFXConfiguration =
    new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20))
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


    private static MotorConfigs coralElevatorMotorConfigs = 
    new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(coralElevatorTalonFXConfiguration)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);

    public static MotorIOTalonFX coralElevatorMotor = new MotorIOTalonFX(coralElevatorMotorConfigs);


    /* CORAL ELEVATOR MOTOR FOLLOWER */
    private static TalonFXConfiguration coralElevatorFollowerTalonFXConfiguration =
    new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20))
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

    private static MotorConfigs coralElevatorFollowerMotorConfigs = 
    new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(coralElevatorFollowerTalonFXConfiguration)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);

    public static MotorIOTalonFX coralElevatorFollowerMotor = new MotorIOTalonFX(coralElevatorFollowerMotorConfigs);


    /* ALGAE ELEVATOR MOTOR */
    public static final double ALGAE_STAGE_UP = 0;
    public static final double ALGAE_STAGE_DOWN = 0;

    private static TalonFXConfiguration algaeElevatorTalonFXConfiguration =
    new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20))
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

    private static MotorConfigs algaeElevatorMotorConfigs = 
    new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(algaeElevatorTalonFXConfiguration)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);
      


    public static MotorIOTalonFX algaeElevatorMotor = new MotorIOTalonFX(algaeElevatorMotorConfigs);
}
