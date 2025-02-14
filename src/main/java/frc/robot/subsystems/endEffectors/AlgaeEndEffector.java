package frc.robot.subsystems.endEffectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class AlgaeEndEffector extends SubsystemBase {    
    private final MotorIO algaeIntakeMotor;
    private final MotorIOInputsAutoLogged algaeIntakeInputs = new MotorIOInputsAutoLogged();

    private final MotorIO algaeWristMotor;
    private final MotorIOInputsAutoLogged algaeWristInputs = new MotorIOInputsAutoLogged();


    public AlgaeEndEffector(MotorIO algaeIntakeMotor, MotorIO algaeWristMotor) {
        this.algaeIntakeMotor = algaeIntakeMotor;
        this.algaeWristMotor = algaeWristMotor;
    }

    @Override
    public void periodic() {
        algaeIntakeMotor.updateInputs(algaeIntakeInputs);
        Logger.processInputs("Algae Intake", algaeIntakeInputs);


        algaeWristMotor.updateInputs(algaeWristInputs);
        Logger.processInputs("Algae Wrist", algaeWristInputs);
    }


    public void setVoltage(double voltage) {
        algaeIntakeMotor.setOpenLoopVoltage(voltage);
    }  

            

    public void setPosition(double position) {
        algaeWristMotor.setMagicalPositionSetpoint(position, 6);
    }
}