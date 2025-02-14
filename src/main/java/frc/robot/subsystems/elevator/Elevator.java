package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class Elevator extends SubsystemBase {    
    private final MotorIO elevatorMotor;
    private final MotorIOInputsAutoLogged elevatorInputs = new MotorIOInputsAutoLogged();


    public Elevator(MotorIO elevatorMotor) {
        this.elevatorMotor = elevatorMotor;
    }

    @Override
    public void periodic() {
        elevatorMotor.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);
    }
    public void setPosition(double position) {
        elevatorMotor.setMagicalPositionSetpoint(position, 6);
    }
}