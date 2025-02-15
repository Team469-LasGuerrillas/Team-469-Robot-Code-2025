package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class HumanPlayerIntake extends SubsystemBase {    
    private static HumanPlayerIntake instance;

    private final MotorIO humanPlayerIntakeMotor;
    private final MotorIOInputsAutoLogged humanPlayerIntakeInputs = new MotorIOInputsAutoLogged();

    public HumanPlayerIntake(MotorIO humanPlayerIntakeMotor) {
        this.humanPlayerIntakeMotor = humanPlayerIntakeMotor;
    }

     public static HumanPlayerIntake createInstance(MotorIO humanPlayerIntakeMotor) {
        instance = new HumanPlayerIntake(humanPlayerIntakeMotor);
        return instance;
    }

    public static HumanPlayerIntake getInstance() {
        if (instance == null) {
            instance = new HumanPlayerIntake(new MotorIO() {});
        }
        return instance;
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