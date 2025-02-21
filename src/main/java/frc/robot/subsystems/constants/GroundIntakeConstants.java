package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.lib.interfaces.motor.CancoderConfigs;
import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.interfaces.sensor.SensorIOCANRange;
import frc.robot.generated.TunerConstants;

public class GroundIntakeConstants {
   
    /* SENSOR CANRANGE */
    SensorIOCANRange CanRange = new SensorIOCANRange(new CANrangeConfiguration(), 0);
    
    /* GROUND WRIST MOTOR */
    public static final double GROUND_WRIST_DOWN = -12;
    public static final double GROUND_WRIST_UP = 12;
    
    private static TalonFXConfiguration groundWristMotorFxConfig = 
    new TalonFXConfiguration()
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

    private static MotorConfigs groundWristMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(groundWristMotorFxConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);

    private static CANcoderConfiguration groundWristCcConfig =
    new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(0)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(0.5)
        );

    private static CancoderConfigs groundWristCancoderConfigs = new CancoderConfigs()
        .withCanId(0)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withCcConfig(groundWristCcConfig);
    
    public static MotorIOTalonFX groundWristMotor = new MotorIOTalonFX(groundWristMotorConfigs, groundWristCancoderConfigs);

    /* GROUND INTAKE MOTOR */
    public static final double GROUND_INTAKE_IN_VOLTAGE = -12;
    public static final double GROUND_INTAKE_OUT_VOLTAGE = 12;
    public static final double GROUND_DEFAULT_VOLTAGE = 0;

    private static TalonFXConfiguration groundIntakeMotorFxConfig = 
    new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(23));

    private static MotorConfigs groundIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus(TunerConstants.kCANBus.toString())
        .withFxConfig(groundIntakeMotorFxConfig)
        .withMaxPositionUnits(469)
        .withMinPositionUnits(0);
    
   public static MotorIOTalonFX groundIntakeMotor = new MotorIOTalonFX(groundIntakeMotorConfigs);    
}
