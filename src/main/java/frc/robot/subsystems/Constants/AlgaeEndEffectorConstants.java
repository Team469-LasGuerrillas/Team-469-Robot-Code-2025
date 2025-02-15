package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;

public class AlgaeEndEffectorConstants {
    private final double algaeIntakeMotorStatorCurrentLimit = 20;
    private final double algaeWristMotorStatorCurrentLimit = 20;

    MotorConfigs algaeIntakeMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469")
        ;

    MotorConfigs algaeWristMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469");

    MotorIOTalonFX algaeIntakeMotor = new MotorIOTalonFX(algaeIntakeMotorConfigs);

    MotorIOTalonFX algaeWristMotor = new MotorIOTalonFX(algaeWristMotorConfigs);
}