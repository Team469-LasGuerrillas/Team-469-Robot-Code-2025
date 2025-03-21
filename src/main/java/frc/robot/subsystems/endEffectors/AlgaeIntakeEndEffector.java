package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.interfaces.sensor.SensorIO;
import frc.lib.interfaces.sensor.SensorIOInputsAutoLogged;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.constants.SensorConstants;
import frc.robot.subsystems.elevator.Elevator;

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
        if (instance == null) throw new Error("Subsystem has not been created");
        return instance;
    }

    @Override
    public void periodic() {
        algaeIntakeMotor.updateInputs(algaeIntakeInputs);
        Logger.processInputs("Algae Intake", algaeIntakeInputs);

        CANRange.updateInputs(CANRangeInputs);
        Logger.processInputs("CANRange Algae", CANRangeInputs);
    }

    public void setVoltage(DoubleSupplier voltage) {
        // if (Elevator.getInstance().getRequestedCoralElevatorPosFromGroundInches() == ElevatorConstants.CORAL_L3_POS
        //      && CoralIntakeEndEffector.getInstance().hasCoral())
        //         algaeIntakeMotor.setOpenLoopVoltage(() -> CoralEndEffectorConstants.CORAL_DEFAULT_VOLTAGE);
        algaeIntakeMotor.setOpenLoopVoltage(voltage);
    }  

    @AutoLogOutput
    public boolean isSeeingAlgae() {
        if (CANRangeInputs.distance < SensorConstants.CAN_RANGE_MAX_ALGAE_VALUE 
            && CANRangeInputs.distance > SensorConstants.CAN_RANGE_MIN_ALGAE_VALUE) return true;
        return false;
    }

    @AutoLogOutput
    public boolean hasAlgae() {
        if (CANRangeInputs.distance < SensorConstants.CAN_RANGE_MIN_ALGAE_VALUE) return true;
        return false;
    }
}