package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class GroundWrist extends SubsystemBase {  
    private static GroundWrist instance;

    private final MotorIO groundWristMotor;
    MotorIOInputsAutoLogged groundWristInputs = new MotorIOInputsAutoLogged();

    public GroundWrist(MotorIO groundWristMotor) {
        this.groundWristMotor = groundWristMotor;
    }

 public static GroundWrist createInstance(MotorIO groundWristMotor) {
        instance = new GroundWrist(groundWristMotor);
        return instance;
    }

    public static GroundWrist getInstance() {
        if (instance == null) throw new Error("Subsystem has not been created");
        return instance;
    }

    @Override
    public void periodic() {
        groundWristMotor.updateInputs(groundWristInputs);
        Logger.processInputs("Ground Wrist", groundWristInputs);
    }

    public void setPosition(double position) {
        groundWristMotor.setMagicalPositionSetpoint(position, 7);
    }
  }
