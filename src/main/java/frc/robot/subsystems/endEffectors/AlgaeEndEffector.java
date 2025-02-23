package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class AlgaeEndEffector extends SubsystemBase {
    private static AlgaeEndEffector instance;

    private final MotorIO algaeIntakeMotor;
    private final MotorIOInputsAutoLogged algaeIntakeInputs = new MotorIOInputsAutoLogged();

    private final MotorIO algaeWristMotor;
    private final MotorIOInputsAutoLogged algaeWristInputs = new MotorIOInputsAutoLogged();

    private AlgaeEndEffector(MotorIO algaeIntakeMotor, MotorIO algaeWristMotor) {
        this.algaeIntakeMotor = algaeIntakeMotor;
        this.algaeWristMotor = algaeWristMotor;

    }

    public static AlgaeEndEffector createInstance(MotorIO algaeIntakeMotor, MotorIO algaeWristMotor) {
        instance = new AlgaeEndEffector(algaeIntakeMotor, algaeWristMotor);
        return instance;
    }

    public static AlgaeEndEffector getInstance() {
        if (instance == null) {
            instance = new AlgaeEndEffector(new MotorIO() {}, new MotorIO() {});
        }
        return instance;
    }

    @Override
    public void periodic() {
        algaeIntakeMotor.updateInputs(algaeIntakeInputs);
        Logger.processInputs("Algae Intake", algaeIntakeInputs);


        algaeWristMotor.updateInputs(algaeWristInputs);
        Logger.processInputs("Algae Wrist", algaeWristInputs);
    }


    public void setVoltage(DoubleSupplier voltage) {
        algaeIntakeMotor.setOpenLoopVoltage(voltage);
    }  

    public void setPosition(double position) {
        algaeWristMotor.setMagicalPositionSetpoint(position, 6);
    }
}