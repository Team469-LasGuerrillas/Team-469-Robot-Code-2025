package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;

public class ClimbConstants {
    private final double climbMotorStatorCurrentLimit = 20;

    MotorConfigs climbMotorConfigs = new MotorConfigs()
        .withCanId(469)
        .withCanBus("469");

    MotorIOTalonFX climbMotor = new MotorIOTalonFX(climbMotorConfigs);
}