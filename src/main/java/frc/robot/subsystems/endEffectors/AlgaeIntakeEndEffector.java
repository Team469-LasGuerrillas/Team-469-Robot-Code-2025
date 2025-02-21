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

public class AlgaeIntakeEndEffector extends SubsystemBase {
    private static AlgaeIntakeEndEffector instance;
    private final SensorIO CANRange;
    private final SensorIOInputsAutoLogged CANRangeInputs = new SensorIOInputsAutoLogged();

    private final MotorIO algaeIntakeMotor;
    private final MotorIOInputsAutoLogged algaeIntakeInputs = new MotorIOInputsAutoLogged();

    private AlgaeIntakeEndEffector(MotorIO algaeIntakeMotor, SensorIO CANRange) {
        this.algaeIntakeMotor = algaeIntakeMotor;
        this.CANRange = CANRange;
    }

    public static AlgaeIntakeEndEffector createInstance(MotorIO algaeIntakeMotor, SensorIO CANRange) {
        instance = new AlgaeIntakeEndEffector(algaeIntakeMotor, CANRange);
        return instance;
    }

    public static AlgaeIntakeEndEffector getInstance() {
        if (instance == null) {
            instance = new AlgaeIntakeEndEffector(new MotorIO() {}, new SensorIO() {});
        }
        return instance;
    }

    @Override
    public void periodic() {
        algaeIntakeMotor.updateInputs(algaeIntakeInputs);
        Logger.processInputs("Algae Intake", algaeIntakeInputs);

        CANRange.updateInputs(CANRangeInputs);
        Logger.processInputs("CANRange", CANRangeInputs);
    }

    public void setVoltage(DoubleSupplier voltage) {
        algaeIntakeMotor.setOpenLoopVoltage(voltage);
    }  

    public boolean hasAlgae() {
        if (CANRangeInputs.distance < SensorConstants.CAN_RANGE_THRESHOLD_VALUE) { 
            return true;
        } else {
            return false;
        }

    }
}