package frc.robot.subsystems.endEffectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class CoralEndEffector extends SubsystemBase {    
    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    public CoralEndEffector(MotorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral End Effector", inputs);
    }

    public void setVoltage(double voltage) {
        io.setOpenLoopVoltage(voltage);
    }   
}
