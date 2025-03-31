package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.interfaces.sensor.SensorIOInputsAutoLogged;
import frc.lib.interfaces.sensor.SensorIO;

public class HumanPlayerIntake extends SubsystemBase {    
    private static HumanPlayerIntake instance;

    private final MotorIO humanPlayerIntakeMotor;
    private final MotorIOInputsAutoLogged humanPlayerIntakeInputs = new MotorIOInputsAutoLogged();
    private final SensorIO beamBreak;
    private final SensorIOInputsAutoLogged beamBreakInputs = new SensorIOInputsAutoLogged();


    public HumanPlayerIntake(MotorIO humanPlayerIntakeMotor, SensorIO beamBreak) {
        this.humanPlayerIntakeMotor = humanPlayerIntakeMotor;
        this.beamBreak = beamBreak;
    }

     public static HumanPlayerIntake createInstance(MotorIO humanPlayerIntakeMotor, SensorIO beamBreak) {
        instance = new HumanPlayerIntake(humanPlayerIntakeMotor, beamBreak);
        return instance;
    }

    public static HumanPlayerIntake getInstance() {
        if (instance == null) throw new Error("Subsystem has not been created");
        return instance;
    }
    @Override
    public void periodic() {
        humanPlayerIntakeMotor.updateInputs(humanPlayerIntakeInputs);
        Logger.processInputs("HumanPlayer Intake", humanPlayerIntakeInputs);

        beamBreak.updateInputs(beamBreakInputs);
        Logger.processInputs("BeamBreak", beamBreakInputs);
    }

    public void setVoltage(DoubleSupplier voltage) {
        humanPlayerIntakeMotor.setOpenLoopVoltage(voltage);
    }   

    public boolean hasCoral() {
        return beamBreakInputs.isCut;
    }
}