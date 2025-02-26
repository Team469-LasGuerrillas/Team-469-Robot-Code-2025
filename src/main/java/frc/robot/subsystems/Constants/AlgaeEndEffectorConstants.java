package frc.robot.subsystems.constants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import frc.lib.interfaces.sensor.SensorIOCANRange;
import frc.robot.generated.TunerConstants;

public class AlgaeEndEffectorConstants {

    /* SENSOR CANRANGE */
    SensorIOCANRange CanRange = new SensorIOCANRange(new CANrangeConfiguration(), 0);

    /* ALGAE INTAKE MOTOR */
    public static final double ALGAE_INTAKE_IN_VOLTAGE = 12;
    public static final double ALGAE_INTAKE_OUT_VOLTAGE = -12;
    public static final double ALGAE_INTAKE_BARGE_OUT_VOLTAGE = -24;
    public static final double ALGAE_INTAKE_DEFAULT_VOLTAGE = 0;
    public static final double ALGAE_EXTENSION_THRESHOLD = 0;

    private static TalonFXConfiguration algaeIntakeMotorFxConfig = 
    new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20));
    
    private static MotorConfigs algaeIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(algaeIntakeMotorFxConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);
    
    public static MotorIOTalonFX algaeIntakeMotor = new MotorIOTalonFX(algaeIntakeMotorConfigs);

    /* ALGAE WRIST MOTOR */
    public static final double ALGAE_WRIST_DEFAULT = 0;
    public static final double ALGAE_WRIST_PROCESSOR = 0;
    public static final double ALGAE_WRIST_BARGE = 0;
    public static final double ALGAE_WRIST_GROUND = 0;
    public static final double ALGAE_WRIST_REEF = 0;
    public static final double IS_ON_TARGET_THRESHOLD = 0.002;
    public static final double ALGAE_WRIST_FEED_FORWARD_VOLTS = 0;

    private static TalonFXConfiguration algaeWristMotorFxConfig = 
    new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        )
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
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(algaeWristMotorFxConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);
    
    private static CANcoderConfiguration algaeWristCcConfig = 
    new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(0)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(0.5)
        );
    
    private static CancoderConfigs algaeWristCancoderConfigs = new CancoderConfigs()
        .withCanId(0)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withCcConfig(algaeWristCcConfig);

    public static MotorIOTalonFX algaeWristMotor = new MotorIOTalonFX(algaeWristMotorConfigs, algaeWristCancoderConfigs);
}