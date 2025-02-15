package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorConfigs;
import frc.lib.interfaces.motor.MotorIOTalonFX;

public class CoralEndEffectorConstants {
    public static final double INTAKE_VOLTAGE = -12;
    public static final double OUTAKE_VOLTAGE = 12;
    public static final double DEFAULT_VOLTAGE = 0;

    private final double coralIntakeMotorStatorCurrentLimit = 20;
    private final double coralWristMotorStatorCurrentLimit = 20;

    MotorConfigs coralIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469");

    MotorConfigs coralWristMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469");

    MotorIOTalonFX coralIntakeMotor = new MotorIOTalonFX(coralIntakeMotorConfigs);

    MotorIOTalonFX coralWristMotor = new MotorIOTalonFX(coralWristMotorConfigs);
}