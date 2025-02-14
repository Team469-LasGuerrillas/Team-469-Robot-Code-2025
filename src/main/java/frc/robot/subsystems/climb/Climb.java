package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class Climb extends SubsystemBase {    
    private final MotorIO climbMotor;
    private final MotorIOInputsAutoLogged climbInputs = new MotorIOInputsAutoLogged();


    public Climb(MotorIO climbMotor) {
        this.climbMotor = climbMotor;
    }

    @Override
    public void periodic() {
        climbMotor.updateInputs(climbInputs);
        Logger.processInputs("Climb", climbInputs);
    }
    public void setPosition(double position) {
        climbMotor.setMagicalPositionSetpoint(position, 6);
    }
}