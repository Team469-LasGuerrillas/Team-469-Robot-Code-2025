package frc.robot.subsystems.endEffectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class CoralEndEffector extends SubsystemBase {
    private static CoralEndEffector instance;  

    private final MotorIO coralIntakeMotor;
    private final MotorIOInputsAutoLogged coralIntakeInputs = new MotorIOInputsAutoLogged();

    private final MotorIO coralWristMotor;
    MotorIOInputsAutoLogged coralWristInputs = new MotorIOInputsAutoLogged();

    public CoralEndEffector(MotorIO coralIntakeMotor, MotorIO coralWristMotor) {
        this.coralIntakeMotor = coralIntakeMotor;
        this.coralWristMotor = coralWristMotor;}

        public static CoralEndEffector createInstance(MotorIO coralIntakeMotor, MotorIO coralWristMotor) {
            instance = new CoralEndEffector(coralIntakeMotor, coralWristMotor);
            return instance;
        }
    
        public static CoralEndEffector getInstance() {
            if (instance == null) {
                instance = new CoralEndEffector(new MotorIO() {}, new MotorIO() {});
            }
            return instance;
        
    }

    @Override
    public void periodic() {
        coralIntakeMotor.updateInputs(coralIntakeInputs);
        Logger.processInputs("Coral Intake", coralIntakeInputs);

        coralWristMotor.updateInputs(coralWristInputs);
        Logger.processInputs("Coral Wrist", coralWristInputs);
    }

    public void setVoltage(double voltage) {
        coralIntakeMotor.setOpenLoopVoltage(voltage);
    }   

    public void setPosition(double position) {
        coralWristMotor.setMagicalPositionSetpoint(position, 7);
    }
}