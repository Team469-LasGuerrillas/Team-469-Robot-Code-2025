package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;

public class Climb extends SubsystemBase {    
    private static Climb instance;

    private final MotorIO climbMotor;
    private final MotorIOInputsAutoLogged climbInputs = new MotorIOInputsAutoLogged();


    public Climb(MotorIO climbMotor) {
        this.climbMotor = climbMotor;
    }
    public static Climb createInstance(MotorIO climbMotor) {
        instance = new Climb(climbMotor);
        return instance;
    }

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb(new MotorIO() {});
        }
        return instance;
    }

    @Override
    public void periodic() {
        climbMotor.updateInputs(climbInputs);
        Logger.processInputs("Climb", climbInputs);
    }
    public void setVoltage(DoubleSupplier voltage) {
        climbMotor.setOpenLoopVoltage(voltage);
    }
}