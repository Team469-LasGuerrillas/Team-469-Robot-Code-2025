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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.lib.interfaces.motor.CancoderConfigs;
import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.interfaces.sensor.SensorIOCANRange;
import frc.robot.generated.TunerConstants;

public class AlgaeEndEffectorConstants {

    /* SENSOR CANRANGE */
    public static SensorIOCANRange CanRange = new SensorIOCANRange(new CANrangeConfiguration(), 1);

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
                .withInverted(InvertedValue.CounterClockwise_Positive)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20));
    
    private static MotorConfigs algaeIntakeMotorConfigs = new MotorConfigs()
        .withCanId(12)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(algaeIntakeMotorFxConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);
    
    public static MotorIOTalonFX algaeIntakeMotor = new MotorIOTalonFX(algaeIntakeMotorConfigs);

    /* ALGAE WRIST MOTOR */
    public static final double ALGAE_WRIST_DEFAULT_POS = 0;
    public static final double ALGAE_WRIST_PROCESSOR_POS = 0;
    public static final double ALGAE_WRIST_BARGE_POS = 0;
    public static final double ALGAE_WRIST_GROUND_POS = 0;
    public static final double ALGAE_WRIST_L2_L3 = 0;
    public static final double IS_ON_TARGET_THRESHOLD = 0.002;
    public static final double VOLTAGE_TO_MAINTAIN_HORIZONTAL = 0.4;
    public static final double ALGAE_WRIST_HORZIONTAL_POS = 0.23;

    private static TalonFXConfiguration algaeWristMotorFxConfig = 
    new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(60))
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
        .withCanId(14)
        .withCanBus(TunerConstants.kCANBus)
        .withFxConfig(algaeWristMotorFxConfig)
        .withRotorToSensorRatio(36.8)
        .withSensorToMechanismRatio(1)
        .withUnitToRotorRatio(36.8)
        .withMaxPositionUnits(0.27)
        .withMinPositionUnits(0);
    
    private static CANcoderConfiguration algaeWristCcConfig = 
    new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(1)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(-0.25)
        );
    
    private static CancoderConfigs algaeWristCancoderConfigs = new CancoderConfigs()
        .withCanId(5)
        .withCanBus(TunerConstants.kCANBus)
        .withCcConfig(algaeWristCcConfig);

    public static MotorIOTalonFX algaeWristMotor = new MotorIOTalonFX(algaeWristMotorConfigs, algaeWristCancoderConfigs);
}