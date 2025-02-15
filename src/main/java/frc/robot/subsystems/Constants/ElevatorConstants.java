package frc.robot.subsystems.constants;

import frc.lib.interfaces.motor.MotorIOTalonFX;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.interfaces.motor.MotorConfigs;

public class ElevatorConstants {
    /* ELEVATOR MOTOR */

    TalonFXConfiguration elevatorTalonFXConfiguration =
    new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(20));

    MotorConfigs elevatorMotorConfigs = 
    new MotorConfigs()
        .withCanId(469)
        .withCanBus("469");

    MotorIOTalonFX elevatorMotor = new MotorIOTalonFX(elevatorMotorConfigs);
}
