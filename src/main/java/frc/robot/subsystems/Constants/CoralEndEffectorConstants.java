package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.lib.interfaces.motor.CancoderConfigs;
import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.interfaces.sensor.SensorIOCANRange;
import frc.robot.generated.TunerConstants;

public class CoralEndEffectorConstants {
    
    /* SENSOR CANRANGE */
    private static final double SIGNAL_STRENGTH_THRESHOLD = 2750;
    private static final double PROXIMITY_DETECTION_THRESHOLD_METERS = 0.08;

    private static CANrangeConfiguration canRangeConfig = new CANrangeConfiguration()
        .withFovParams(
            new FovParamsConfigs()
            .withFOVRangeX(27)
            .withFOVRangeY(15)
        )
        .withProximityParams(
            new ProximityParamsConfigs()
            .withMinSignalStrengthForValidMeasurement(SIGNAL_STRENGTH_THRESHOLD)
            .withProximityThreshold(PROXIMITY_DETECTION_THRESHOLD_METERS)
        );

    public static SensorIOCANRange canRange = new SensorIOCANRange(canRangeConfig, 1);

    /* CORAL WRIST MOTOR */
    public static final double CORAL_L4_POS = 0.69;
    public static final double CORAL_L3_POS = 0.675;
    public static final double CORAL_L2_POS = 0.675;
    public static final double CORAL_L1_POS = 0.61;
    public static final double CORAL_GROUND_INTAKE_POS = 0;
    public static final double CORAL_HP_INTAKE_POS = 0.14; // 0.136
    public static final double CORAL_WRIST_DEFAULT_POS = 0.4;
    public static final double CORAL_PROCESSOR_POS = CORAL_L1_POS;

    public static final double CORAL_WRIST_FLIP_THRESHOLD_HIGH = 0.5;
    public static final double CORAL_WRIST_FLIP_THRESHOLD_LOW = 0.4;
    public static final double CORAL_BARGE_POS = CORAL_WRIST_FLIP_THRESHOLD_HIGH;
    public static final double IDLE_WRIST_THRESHOLD = 0.45;
    public static final double IS_ON_TARGET_THRESHOLD = 0.015;
    public static final double HORIZONTAL_POSITION = 0.045;
    public static final double VOLTAGE_TO_MAINTAIN_HORIZONTAL_WO_CORAL = 1.4;
    public static final double VOLTAGE_TO_MAINTAIN_HORIZONTAL_W_CORAL = 3.25;
    
    public static final double NUM_OF_ON_TARGET_LOOPS = 5;
    
    private static TalonFXConfiguration coralWristMotorFxConfig = 
    new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive) // TODO: CLOCKWISE OR COUNTERCLOCKWISE
                .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80) // TODO: Determine Current Limits
        ).withSlot0(
            new Slot0Configs() // TODO: PID Tuning
            .withKS(0)
            .withKV(0)
            .withKP(100)
            .withKI(0)
            .withKD(0)
        ).withSlot1(
            new Slot1Configs() // TODO: PID Tuning
            .withKS(0)
            .withKV(0)
            .withKP(125)
            .withKI(0)
            .withKD(0)
        )
        .withMotionMagic(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(80)
            .withMotionMagicAcceleration(24)
            .withMotionMagicJerk(100));

    private static MotorConfigs coralWristMotorConfigs = new MotorConfigs()
        .withCanId(15)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(coralWristMotorFxConfig)
        .withRotorToSensorRatio(16.1904761905)
        .withSensorToMechanismRatio(1)
        .withUnitToRotorRatio(1)
        .withMinPositionUnits(0.01)
        .withMaxPositionUnits(0.7);

    private static CANcoderConfiguration coralWristCcConfig = 
    new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(1)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive) // TODO: CANCoder Direction
            .withMagnetOffset(0.108) // TODO: Magnet Offset
        );

    private static CancoderConfigs coralWristCancoderConfigs = new CancoderConfigs()
        .withCanId(4)
        .withCanBus(TunerConstants.kCANBus)
        .withCcConfig(coralWristCcConfig);
    
    public static MotorIOTalonFX coralWristMotor = new MotorIOTalonFX(coralWristMotorConfigs, coralWristCancoderConfigs);

    /* CORAL INTAKE MOTOR */
    public static final double CORAL_INTAKE_IN_VOLTAGE = -4.5;
    public static final double CORAL_INTAKE_OUT_VOLTAGE = 12;
    public static final double CORAL_DEFAULT_VOLTAGE = 0;
    public static final double CORAL_FINAL_RETAINING_VOLTAGE = -2;

    private static TalonFXConfiguration coralIntakeMotorFxConfig = 
    new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(120));

    private static MotorConfigs coralIntakeMotorConfigs = new MotorConfigs()
        .withCanId(13)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(coralIntakeMotorFxConfig);

   public static MotorIOTalonFX coralIntakeMotor = new MotorIOTalonFX(coralIntakeMotorConfigs);
}