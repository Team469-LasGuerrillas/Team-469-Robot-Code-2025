package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.interfaces.sensor.SensorIO;
import frc.lib.interfaces.sensor.SensorIOInputsAutoLogged;
import frc.robot.subsystems.constants.SensorConstants;

public class CoralIntakeEndEffector extends SubsystemBase {
    private static CoralIntakeEndEffector instance;  
    private final SensorIO CANRange;
    private final SensorIOInputsAutoLogged CANRangeInputs = new SensorIOInputsAutoLogged();


    private final MotorIO coralIntakeMotor;
    private final MotorIOInputsAutoLogged coralIntakeInputs = new MotorIOInputsAutoLogged();

    public CoralIntakeEndEffector(MotorIO coralIntakeMotor, SensorIO CANRange) {
        this.coralIntakeMotor = coralIntakeMotor;
        this.CANRange = CANRange;
    }

        public static CoralIntakeEndEffector createInstance(MotorIO coralIntakeMotor, SensorIO CANRange) {
            instance = new CoralIntakeEndEffector(coralIntakeMotor, CANRange);
            return instance;
        }
    
        public static CoralIntakeEndEffector getInstance() {
            if (instance == null) {
                instance = new CoralIntakeEndEffector(new MotorIO() {}, new SensorIO() {});
            }
            return instance;
        
    }

    @Override
    public void periodic() {
        coralIntakeMotor.updateInputs(coralIntakeInputs);
        Logger.processInputs("Coral Intake", coralIntakeInputs);

        CANRange.updateInputs(CANRangeInputs);
        Logger.processInputs("CANRange", CANRangeInputs);
    }

    public void setVoltage(DoubleSupplier voltage) {
        coralIntakeMotor.setOpenLoopVoltage(voltage);
    }   
    
    public boolean hasCoral() {
        if (CANRangeInputs.distance < SensorConstants.CAN_RANGE_THRESHOLD_VALUE) {
            return true;
        } else {
            return false;
        }
    }
}