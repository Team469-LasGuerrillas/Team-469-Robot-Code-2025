package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class HumanPlayerIntake extends SubsystemBase {    
    private final MotorIO humanPlayerIntakeMotor;
    private final MotorIOInputsAutoLogged humanPlayerIntakeInputs = new MotorIOInputsAutoLogged();

    public HumanPlayerIntake(MotorIO humanPlayerIntakeMotor) {
        this.humanPlayerIntakeMotor = humanPlayerIntakeMotor;
    }

    @Override
    public void periodic() {
        humanPlayerIntakeMotor.updateInputs(humanPlayerIntakeInputs);
        Logger.processInputs("HumanPlayer Intake", humanPlayerIntakeInputs);
    }

    public void setVoltage(double voltage) {
        humanPlayerIntakeMotor.setOpenLoopVoltage(voltage);
    }   
}