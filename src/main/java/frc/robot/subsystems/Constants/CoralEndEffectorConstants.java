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
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.lib.interfaces.motor.CancoderConfigs;
import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.interfaces.sensor.SensorIOBeamBreak;
import frc.lib.interfaces.sensor.SensorIOCANRange;
import frc.robot.generated.TunerConstants;

public class CoralEndEffectorConstants {
    
    /* SENSOR CANRANGE */
    SensorIOCANRange CanRange = new SensorIOCANRange(new CANrangeConfiguration(), 0);

    /* CORAL WRIST MOTOR */
    public static final double CORAL_L4_POSITION = -12;
    public static final double CORAL_L3_POSITION = 12;
    public static final double CORAL_L2_POSITION = 0;
    public static final double CORAL_L1_POSITION = 0;
    public static final double CORAL_GROUND_INTAKE_POSITION = 0;
    public static final double CORAL_WRIST_DEFAULT_POSITION = 0;
    public static final double CORAL_HP_INTAKE_POSITION = 0;
    public static final double CORAL_WRIST_FLIP_THRESHOLD = 0;
    public static final double IS_ON_TARGET_THRESHOLD = 0.002;

    private static TalonFXConfiguration coralWristMotorFxConfig = 
    new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(23)
        ).withSlot0(
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

    private static MotorConfigs coralWristMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(coralWristMotorFxConfig);

    private static CANcoderConfiguration coralWristCcConfig = 
    new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(0)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(0.5)
        );

    private static CancoderConfigs coralWristCancoderConfigs = new CancoderConfigs()
        .withCanId(0)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withCcConfig(coralWristCcConfig);
    
    public static MotorIOTalonFX coralWristMotor = new MotorIOTalonFX(coralWristMotorConfigs, coralWristCancoderConfigs);

    /* CORAL INTAKE MOTOR */
    public static final double CORAL_INTAKE_IN_VOLTAGE = -12;
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
            .withStatorCurrentLimit(23));

    private static MotorConfigs coralIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(coralIntakeMotorFxConfig);

   public static MotorIOTalonFX coralIntakeMotor = new MotorIOTalonFX(coralIntakeMotorConfigs);
}