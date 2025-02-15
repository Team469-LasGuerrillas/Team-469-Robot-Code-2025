package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class Elevator extends SubsystemBase {    
    private static Elevator instance;

    private final MotorIO elevatorMotor;
    private final MotorIOInputsAutoLogged elevatorInputs = new MotorIOInputsAutoLogged();


    public Elevator(MotorIO elevatorMotor) {
        this.elevatorMotor = elevatorMotor;
    }

    public static Elevator createInstance(MotorIO elevatorMotor) {
        instance = new Elevator(elevatorMotor);
        return instance;
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator(new MotorIO() {});
        }
        return instance;
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