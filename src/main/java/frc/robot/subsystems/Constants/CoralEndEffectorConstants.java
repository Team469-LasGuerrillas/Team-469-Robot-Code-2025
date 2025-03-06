package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.lib.interfaces.motor.CancoderConfigs;
import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.interfaces.sensor.SensorIOBeamBreak;
import frc.lib.interfaces.sensor.SensorIOCANRange;
import frc.robot.generated.TunerConstants;

public class CoralEndEffectorConstants {
    
    /* SENSOR CANRANGE */
    public static SensorIOCANRange CanRange = new SensorIOCANRange(new CANrangeConfiguration(), 0);

    /* CORAL WRIST MOTOR */
    public static final double CORAL_L4_POS = -12;
    public static final double CORAL_L3_POS = 12;
    public static final double CORAL_L2_POS = 0;
    public static final double CORAL_L1_POS = 0;
    public static final double CORAL_GROUND_INTAKE_POS = 0;
    public static final double CORAL_HP_INTAKE_POS = 0.2025;
    public static final double CORAL_WRIST_DEFAULT_POS = 0.45;
    public static final double CORAL_WRIST_FLIP_THRESHOLD = 0;
    public static final double IS_ON_TARGET_THRESHOLD = 0.002;
    public static final double HORIZONTAL_POSITION = 0.045;
    public static final double VOLTAGE_TO_MAINTAIN_HORIZONTAL = 0.625;
    
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
            .withStatorCurrentLimit(60) // TODO: Determine Current Limits
        ).withSlot0(
            new Slot0Configs() // TODO: PID Tuning
            .withKS(0)
            .withKV(0)
            .withKP(11)
            .withKI(0)
            .withKD(0))
        .withMotionMagic(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(80)
            .withMotionMagicAcceleration(90));

    private static MotorConfigs coralWristMotorConfigs = new MotorConfigs()
        .withCanId(15)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(coralWristMotorFxConfig)
        .withRotorToSensorRatio(1)
        .withSensorToMechanismRatio(1)
        .withUnitToRotorRatio(1 / 16.1904761905)
        .withMinPositionUnits(0.01)
        .withMaxPositionUnits(0.6);

    // private static CANcoderConfiguration coralWristCcConfig = 
    // new CANcoderConfiguration()
    //     .withMagnetSensor(
    //         new MagnetSensorConfigs()
    //         .withAbsoluteSensorDiscontinuityPoint(1)
    //         .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive) // TODO: CANCoder Direction
    //         .withMagnetOffset(0.12) // TODO: Magnet Offset
    //     );

    // private static CancoderConfigs coralWristCancoderConfigs = new CancoderConfigs()
    //     .withCanId(4)
    //     .withCanBus(TunerConstants.kCANBus)
    //     .withCcConfig(coralWristCcConfig);
    
    public static MotorIOTalonFX coralWristMotor = new MotorIOTalonFX(coralWristMotorConfigs);

    /* CORAL INTAKE MOTOR */
    public static final double CORAL_INTAKE_IN_VOLTAGE = -2.5;
    public static final double CORAL_INTAKE_OUT_VOLTAGE = 12;
    public static final double CORAL_DEFAULT_VOLTAGE = 0;

    private static TalonFXConfiguration coralIntakeMotorFxConfig = 
    new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(50));

    private static MotorConfigs coralIntakeMotorConfigs = new MotorConfigs()
        .withCanId(13)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(coralIntakeMotorFxConfig);

   public static MotorIOTalonFX coralIntakeMotor = new MotorIOTalonFX(coralIntakeMotorConfigs);
}