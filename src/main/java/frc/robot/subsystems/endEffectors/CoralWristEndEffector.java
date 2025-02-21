package frc.robot.subsystems.endEffectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class CoralWristEndEffector extends SubsystemBase {
    private static CoralWristEndEffector instance;  

    private final MotorIO coralWristMotor;
    MotorIOInputsAutoLogged coralWristInputs = new MotorIOInputsAutoLogged();

    public CoralWristEndEffector(MotorIO coralWristMotor) {
        this.coralWristMotor = coralWristMotor;}

        public static CoralWristEndEffector createInstance(MotorIO coralWristMotor) {
            instance = new CoralWristEndEffector(coralWristMotor);
            return instance;
        }
    
        public static CoralWristEndEffector getInstance() {
            if (instance == null) {
                instance = new CoralWristEndEffector(new MotorIO() {});
            }
            return instance;
        
    }

    @Override
    public void periodic() {
        coralWristMotor.updateInputs(coralWristInputs);
        Logger.processInputs("Coral Wrist", coralWristInputs);
    }

    public void setPosition(double position) {
        coralWristMotor.setMagicalPositionSetpoint(position, 7);
    }
}