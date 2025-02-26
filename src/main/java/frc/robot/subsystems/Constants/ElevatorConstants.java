package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.networktables.BooleanEntry;
import frc.lib.interfaces.motor.MotorConfigs;

public class ElevatorConstants {

    // TODO: Go to Onshape and get the correct values for all of the following 3 static final variables
    public static final double MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES = 6; 
    public static final double CARRIAGE_HEIGHT = 2;
    public static final double GROUND_TO_CORAL_REST_POS_INCHES = 6;
    public static final double GROUND_TO_ALGAE_REST_POS_INCHES = 4;

    public static final double MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP = GROUND_TO_CORAL_REST_POS_INCHES + 0.5;
    public static final double MIN_ELEVATOR_HEIGHT_FOR_ALGAE_OUT = 5;

    public static final double IS_ON_TARGET_THRESHOLD = 0.002;
    
    /* CORAL ELEVATOR MOTOR */
    public static final double CORAL_RESTING_POS = 0;
    public static final double CORAL_GROUND_INTAKE = 1;
    public static final double CORAL_HUMAN_PLAYER_INTAKE = 4;
    public static final double CORAL_L1 = 2;
    public static final double CORAL_L2 = 3;
    public static final double CORAL_L3 = 5;
    public static final double CORAL_L4 = 6;
    public static final double CORAL_BARGE = 7;
    
    public static final double FEEDFORWARD_VOLTS = 0;    

    private static TalonFXConfiguration coralElevatorTalonFXConfiguration =
    new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(10))
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
        .withCanId(8)
        .withCanBus(TunerConstants.kCANBus.toString())
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
            .withStatorCurrentLimit(10))
            .withSlot0(
                new Slot0Configs()
                .withKS(0)
                .withKV(0)
                .withKP(0)
                .withKI(0)
                .withKD(0)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80)
                .withMotionMagicAcceleration(90));

    private static MotorConfigs coralElevatorFollowerMotorConfigs = 
    new MotorConfigs()
        .withCanId(9)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(coralElevatorFollowerTalonFXConfiguration)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);

    public static MotorIOTalonFX coralElevatorFollowerMotor = new MotorIOTalonFX(coralElevatorFollowerMotorConfigs);

    /* ALGAE ELEVATOR MOTOR */
    public static final double ALGAE_PROCESSOR = 0;
    public static final double ALGAE_RESTING_POS = 1;
    public static final double ALGAE_L2 = 2;
    public static final double ALGAE_L3 = 3;
    public static final double ALGAE_BARGE = 4;

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
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(algaeElevatorTalonFXConfiguration)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);
      


    public static MotorIOTalonFX algaeElevatorMotor = new MotorIOTalonFX(algaeElevatorMotorConfigs);
}
