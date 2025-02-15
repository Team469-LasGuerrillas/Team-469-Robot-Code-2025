package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;

public class CoralEndEffectorConstants {
  

    /* CORAL WRIST MOTOR */
    public static final double CORAL_L4_POSITION = -12;
    public static final double CORAL_L3_POSITION = 12;
    public static final double CORAL_L2_POSITION = 0;
    public static final double CORAL_L1_POSITION = 0;
    public static final double CORAL_GROUND_INTAKE_POSITION = 0;
    public static final double CORAL_HP_INTAKE_POSITION = 0;

    private static TalonFXConfiguration coralWristMotorFxConfig = 
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

    private static MotorConfigs coralWristMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(coralWristMotorFxConfig);
    
    public static MotorIOTalonFX coralWristMotor = new MotorIOTalonFX(coralWristMotorConfigs);

    /* CORAL INTAKE MOTOR */
    public static final double CORAL_INTAKE_IN_VOLTAGE = -12;
    public static final double CORAL_INTAKE_OUT_VOLTAGE = 12;
    public static final double CORAL_DEFAULT_VOLTAGE = 0;

    private static TalonFXConfiguration coralIntakeMotorFxConfig = 
    new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(23));

    private static MotorConfigs coralIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        .withFxConfig(coralIntakeMotorFxConfig);
    
   public static MotorIOTalonFX coralIntakeMotor = new MotorIOTalonFX(coralIntakeMotorConfigs);    
}