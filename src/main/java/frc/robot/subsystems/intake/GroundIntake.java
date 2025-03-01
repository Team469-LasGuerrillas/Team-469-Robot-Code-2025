package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.interfaces.sensor.SensorIO;
import frc.lib.interfaces.sensor.SensorIOInputsAutoLogged;
import frc.robot.subsystems.constants.SensorConstants;

public class GroundIntake extends SubsystemBase {  
    private static GroundIntake instance;
    private final SensorIO CANRange;
    private final SensorIOInputsAutoLogged CANRangeInputs = new SensorIOInputsAutoLogged();

    private final MotorIO groundIntakeMotor;
    private final MotorIOInputsAutoLogged groundIntakeInputs = new MotorIOInputsAutoLogged();

    public GroundIntake(MotorIO groundIntakeMotor, SensorIO CANRange) {
        this.groundIntakeMotor = groundIntakeMotor;
        this.CANRange = CANRange;
    }

 public static GroundIntake createInstance(MotorIO groundIntakeMotor, SensorIO CANRange) {
        instance = new GroundIntake(groundIntakeMotor, CANRange);
        return instance;
    }

    public static GroundIntake getInstance() {
        if (instance == null) throw new Error("Subsystem has not been created");
        return instance;
    }

    @Override
    public void periodic() {
        groundIntakeMotor.updateInputs(groundIntakeInputs);
        Logger.processInputs("Ground Intake", groundIntakeInputs);

        CANRange.updateInputs(CANRangeInputs);
        Logger.processInputs("CANRange", CANRangeInputs);
    }
    

    public void setVoltage(DoubleSupplier voltage) {
        groundIntakeMotor.setOpenLoopVoltage(voltage);
    }   

     public boolean hasCoral() {
        if (CANRangeInputs.distance < SensorConstants.CAN_RANGE_THRESHOLD_VALUE) { 
            return true;
        } else {
            return false;
        }
    }
}
