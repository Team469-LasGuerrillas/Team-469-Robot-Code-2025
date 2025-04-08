package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.interfaces.sensor.SensorIO;
import frc.lib.interfaces.sensor.SensorIOInputsAutoLogged;
import frc.lib.util.FieldLayout;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.drive.Drive;

public class CoralIntakeEndEffector extends SubsystemBase {
    private static CoralIntakeEndEffector instance;  
    private final SensorIO CANRange;
    private final SensorIOInputsAutoLogged CANRangeInputs = new SensorIOInputsAutoLogged();


    private final MotorIO coralIntakeMotor;
    private final MotorIOInputsAutoLogged coralIntakeInputs = new MotorIOInputsAutoLogged();

    private DoubleSupplier requestedVoltage = () -> 0;

    public CoralIntakeEndEffector(MotorIO coralIntakeMotor, SensorIO CANRange) {
        this.coralIntakeMotor = coralIntakeMotor;
        this.CANRange = CANRange;
    }

        public static CoralIntakeEndEffector createInstance(MotorIO coralIntakeMotor, SensorIO CANRange) {
            instance = new CoralIntakeEndEffector(coralIntakeMotor, CANRange);
            return instance;
        }
    
        public static CoralIntakeEndEffector getInstance() {
            if (instance == null) throw new Error("Subsystem has not been created");
            return instance;
        
    }

    @Override
    public void periodic() {
        coralIntakeMotor.updateInputs(coralIntakeInputs);
        Logger.processInputs("Coral Intake", coralIntakeInputs);

        CANRange.updateInputs(CANRangeInputs);
        Logger.processInputs("CANRange Coral", CANRangeInputs);

        coralIntakeMotor.setOpenLoopVoltage(requestedVoltage);

    }

    public void setAutoIntake() {
        double result;

        if (
            Drive.getInstance().getPose().getTranslation().getDistance(FieldLayout.HP_0) < FieldLayout.HP_ZONE
            || Drive.getInstance().getPose().getTranslation().getDistance(FieldLayout.HP_1) < FieldLayout.HP_ZONE
            || Drive.getInstance().getPose().getTranslation().getDistance(FieldLayout.HP_2) < FieldLayout.HP_ZONE
            || Drive.getInstance().getPose().getTranslation().getDistance(FieldLayout.HP_3) < FieldLayout.HP_ZONE
        ) result = CoralEndEffectorConstants.CORAL_INTAKE_IN_VOLTAGE;
        else
            result = CoralEndEffectorConstants.CORAL_DEFAULT_VOLTAGE;

        requestedVoltage = () -> result;
    }

    public void setVoltage(DoubleSupplier voltage) {
        requestedVoltage = voltage;
    }   

    public void setDefault() {
        if (!hasCoral()) setVoltage(() -> CoralEndEffectorConstants.CORAL_DEFAULT_VOLTAGE);
        else setVoltage(() -> 0);
    }

    public void setOutakeVoltage() {
        if (CoralWristEndEffector.getInstance().getRequestedPosition() 
            == CoralEndEffectorConstants.CORAL_L1_POS)
            setVoltage(() -> CoralEndEffectorConstants.CORAL_L1_INTAKE_OUT_VOLTAGE);
        else setVoltage(() -> CoralEndEffectorConstants.CORAL_INTAKE_OUT_VOLTAGE);
    }
    
    @AutoLogOutput
    public boolean hasCoral() {
        return CANRangeInputs.isCut;
    }
}