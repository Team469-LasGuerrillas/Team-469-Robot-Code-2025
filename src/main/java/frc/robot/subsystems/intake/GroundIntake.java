package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class GroundIntake extends SubsystemBase {    
    private final MotorIO groundIntakeMotor;
    private final MotorIOInputsAutoLogged groundIntakeInputs = new MotorIOInputsAutoLogged();

    private final MotorIO groundWristMotor;
    MotorIOInputsAutoLogged groundWristInputs = new MotorIOInputsAutoLogged();

    public GroundIntake(MotorIO groundIntakeMotor, MotorIO groundWristMotor) {
        this.groundIntakeMotor = groundIntakeMotor;
        this.groundWristMotor = groundWristMotor;
    }

    @Override
    public void periodic() {
        groundIntakeMotor.updateInputs(groundIntakeInputs);
        Logger.processInputs("Ground Intake", groundIntakeInputs);

        groundWristMotor.updateInputs(groundWristInputs);
        Logger.processInputs("Ground Wrist", groundWristInputs);
    }

    public void setVoltage(double voltage) {
        groundIntakeMotor.setOpenLoopVoltage(voltage);
    }   

    public void setPosition(double position) {
        groundWristMotor.setMagicalPositionSetpoint(position, 7);
    }
}