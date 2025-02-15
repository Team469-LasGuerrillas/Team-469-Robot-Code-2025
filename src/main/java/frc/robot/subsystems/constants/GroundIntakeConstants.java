package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;

public class GroundIntakeConstants {
   
    /* GROUND WRIST MOTOR */
    public static final double GROUND_INTAKE_DOWN = -12;
    public static final double GROUND_INTAKE_UP = 12;
    
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
        .withCanBus("469")
        .withFxConfig(groundWristMotorFxConfig);
    
    public static MotorIOTalonFX groundWristMotor = new MotorIOTalonFX(groundWristMotorConfigs);

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
        .withCanBus("469")
        .withFxConfig(groundIntakeMotorFxConfig);
    
   public static MotorIOTalonFX groundIntakeMotor = new MotorIOTalonFX(groundIntakeMotorConfigs);    
}
