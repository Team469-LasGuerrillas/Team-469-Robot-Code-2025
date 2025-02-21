package frc.robot.subsystems.endEffectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class AlgaeWristEndEffector extends SubsystemBase {
    private static AlgaeWristEndEffector instance;

    private final MotorIO algaeWristMotor;
    private final MotorIOInputsAutoLogged algaeWristInputs = new MotorIOInputsAutoLogged();

    private AlgaeWristEndEffector(MotorIO algaeWristMotor) {
        this.algaeWristMotor = algaeWristMotor;
    }

    public static AlgaeWristEndEffector createInstance(MotorIO algaeWristMotor) {
        instance = new AlgaeWristEndEffector(algaeWristMotor);
        return instance;
    }

    public static AlgaeWristEndEffector getInstance() {
        if (instance == null) {
            instance = new AlgaeWristEndEffector(new MotorIO() {});
        }
        return instance;
    }

    @Override
    public void periodic() {
        algaeWristMotor.updateInputs(algaeWristInputs);
        Logger.processInputs("Algae Wrist", algaeWristInputs);
    }

    public void setPosition(double position) {
        algaeWristMotor.setMagicalPositionSetpoint(position, 6);
    }
}

