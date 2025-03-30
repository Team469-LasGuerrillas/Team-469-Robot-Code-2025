package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.interfaces.motor.MotorConfigs;

public class ElevatorConstants {

    public static final double MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES = 39.6; 
    public static final double CARRIAGE_HEIGHT = 6; // 7 inch Carriage Height + 1 inch Tolerance
    public static final double GROUND_TO_CORAL_REST_POS_INCHES = 12.125;
    public static final double GROUND_TO_ALGAE_REST_POS_INCHES = 5.125;

    public static final double MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_HIGH = GROUND_TO_CORAL_REST_POS_INCHES + 5;
    public static final double MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_LOW = GROUND_TO_CORAL_REST_POS_INCHES + 1.5 + 0.75; // 0.75 is for Reduced tolerance for flip -- @ Detroit
    public static final double MIN_ELEVATOR_HEIGHT_FOR_ALGAE_OUT = GROUND_TO_CORAL_REST_POS_INCHES + 5;
    public static final double MAX_ELEVATOR_HEIGHT_FOR_CORAL_IDLE = GROUND_TO_CORAL_REST_POS_INCHES + 3;

    public static final double IS_ON_TARGET_THRESHOLD = 0.25;

    /* CORAL ELEVATOR MOTOR FOLLOWER */
    private static TalonFXConfiguration coralElevatorFollowerFxConfiguration = 
    new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );

    private static MotorConfigs coralElevatorFollowerMotorConfigs = 
    new MotorConfigs()
        .withCanId(9)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(coralElevatorFollowerFxConfiguration);

    public static MotorIOTalonFX coralElevatorFollowerMotor = new MotorIOTalonFX(coralElevatorFollowerMotorConfigs);

    /* CORAL ELEVATOR MOTOR */
    public static final double CORAL_DEFAULT_POS = GROUND_TO_CORAL_REST_POS_INCHES;
    public static final double CORAL_GROUND_INTAKE_POS = 1;
    public static final double CORAL_HUMAN_PLAYER_INTAKE_POS = CORAL_DEFAULT_POS;
    public static final double CORAL_L1_POS = 24;
    public static final double CORAL_L2_POS = 38;
    public static final double CORAL_L3_POS = 54.5;
    public static final double CORAL_L4_POS = 81;
    public static final double CORAL_PROCESSOR_POS = CORAL_DEFAULT_POS; // 16.469
    public static final double CORAL_BARGE_POS = 88.5;

    public static final double CORAL_VELOCITY = 100; // 60
    public static final double CORAL_SLOW_VELOCITY = 10;
    public static final double CORAL_ACCELERATION = 350; // 350 250

    public static final double CORAL_SLOW_ACCELERATION = 469469469469469469469469469.0;
    public static final double CORAL_JERK = 0;
    
    public static final double CORAL_FEEDFORWARDS_WHEN_ZERO = 0;
    public static final double CORAL_FEEDFORWARD_VOLTS_L0 = 0.4625;
    public static final double CORAL_FEEDFORWARD_VOLTS_L1 = 0.65;
    public static final double CORAL_FEEDFORWARD_VOLTS_L2 = 0.725; 
    public static final double ALGAE_FEEDFORWARD_VOLTS = 0.35;

    public static final double CORAL_FEEDFORWARDS_HEIGHT_ZERO = CORAL_DEFAULT_POS + 2;
    public static final double CORAL_FEEDFORWARDS_HEIGHT_L0 = 30.96;
    public static final double CORAL_FEEDFORWARDS_HEIGHT_L1 = 61.99;

    public static final double CORAL_SLOW_UPPER = 53;
    public static final double CORAL_SLOW_LOWER = 26;

    public static final double DYNAMIC_ELEVATOR_HEIGHT_MAGIC_NUMBER = 1;
    public static final double DYNAMIC_ELEVATOR_CLAMP_RANGE = 0; // 4.69
    
    private static TalonFXConfiguration coralElevatorTalonFXConfiguration =
    new TalonFXConfiguration()
    .withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
    )
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(100))
              .withSlot0(
            new Slot0Configs()
                .withKS(0)
                .withKV(0)
                .withKP(4)
                .withKI(0)
                .withKD(0.1)
            )
            .withSlot1(new Slot1Configs()
                .withKS(0)
                .withKV(0)
                .withKP(3)
                .withKI(0)
                .withKD(0)
            );

    private static MotorConfigs coralElevatorMotorConfigs = 
    new MotorConfigs()
        .withCanId(8)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(coralElevatorTalonFXConfiguration)
        .withUnitToRotorRatio(1.3463968515)
        .withMaxPositionUnits(88.9)
        .withMinPositionUnits(GROUND_TO_CORAL_REST_POS_INCHES);

    public static MotorIOTalonFX coralElevatorMotor = new MotorIOTalonFX(coralElevatorMotorConfigs, coralElevatorFollowerMotor);

    /* ALGAE ELEVATOR MOTOR */
    public static final double ALGAE_DEFAULT_POS = GROUND_TO_ALGAE_REST_POS_INCHES;
    public static final double ALGAE_PROCESSOR_POS = ALGAE_DEFAULT_POS; // 9.469
    public static final double ALGAE_L2_POS = 29.5;
    public static final double ALGAE_L3_POS = 44;
    public static final double ALGAE_BARGE_POS = CORAL_BARGE_POS - 11;
    public static final double ALGAE_GROUND_POS = 5;

    private static TalonFXConfiguration algaeElevatorTalonFXConfiguration =
    new TalonFXConfiguration()
    .withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
    )
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(60))
            .withSlot0(
                new Slot0Configs()
                .withKS(0)
                .withKV(0)
                .withKP(1.25)
                .withKI(0)
                .withKD(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(100)
                .withMotionMagicAcceleration(150));

    private static MotorConfigs algaeElevatorMotorConfigs = 
    new MotorConfigs()
        .withCanId(11)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(algaeElevatorTalonFXConfiguration)
        .withUnitToRotorRatio(0.3333578871)
        .withMaxPositionUnits(22.75)
        .withMinPositionUnits(GROUND_TO_ALGAE_REST_POS_INCHES);

    public static MotorIOTalonFX algaeElevatorMotor = new MotorIOTalonFX(algaeElevatorMotorConfigs);
}
