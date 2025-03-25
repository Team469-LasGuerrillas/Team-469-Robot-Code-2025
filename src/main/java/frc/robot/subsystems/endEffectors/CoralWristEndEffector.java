package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class CoralWristEndEffector extends SubsystemBase {
    private static CoralWristEndEffector instance;  

    private final MotorIO coralWristMotor;
    MotorIOInputsAutoLogged coralWristInputs = new MotorIOInputsAutoLogged();

    private DoubleSupplier requestedPosition = () -> CoralEndEffectorConstants.CORAL_WRIST_DEFAULT_POS;

    public CoralWristEndEffector(MotorIO coralWristMotor) {
        this.coralWristMotor = coralWristMotor;
        this.coralWristMotor.setCurrentPosition(0);
    }

    public static CoralWristEndEffector createInstance(MotorIO coralWristMotor) {
        instance = new CoralWristEndEffector(coralWristMotor);
        return instance;
    }
    
    public static CoralWristEndEffector getInstance() {
        if (instance == null) throw new Error("Subsystem has not been created");
        return instance;
    }

    @Override
    public void periodic() {
        coralWristMotor.updateInputs(coralWristInputs);
        Logger.processInputs("Coral Wrist", coralWristInputs);

        double appliedFF = CoralEndEffectorConstants.VOLTAGE_TO_MAINTAIN_HORIZONTAL_WO_CORAL;
        coralWristMotor.setSlot(0);

        if (CoralIntakeEndEffector.getInstance().hasCoral()) {
            appliedFF = CoralEndEffectorConstants.VOLTAGE_TO_MAINTAIN_HORIZONTAL_W_CORAL;
            coralWristMotor.setSlot(1);
        }

        coralWristMotor.setMagicalPositionSetpoint(
            getClosestAllowedPosition(
                requestedPosition.getAsDouble()), 
                Math.cos((coralWristInputs.unitPosition
                 - CoralEndEffectorConstants.HORIZONTAL_POSITION) * Math.PI * 2) * appliedFF);
    }

    private double getClosestAllowedPosition(double targetPosition) {
        if (Elevator.getInstance().getCurrentCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_HIGH
            && targetPosition < CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_HIGH) {
            return CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_HIGH;
        } else if (Elevator.getInstance().getCurrentCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_LOW
            && targetPosition < CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_LOW) {
            return CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_LOW;
        } else if (Elevator.getInstance().getCurrentCoralElevatorPosFromGroundInches() <= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_IDLE
            && targetPosition > CoralEndEffectorConstants.IDLE_WRIST_THRESHOLD) {
            return CoralEndEffectorConstants.IDLE_WRIST_THRESHOLD;
        } else {
            return targetPosition;
        }
    }

    public void setPosition(DoubleSupplier position) {
        requestedPosition = position;
    }

    public void setDefault() {
        if (CoralIntakeEndEffector.getInstance().hasCoral()) requestedPosition = () -> CoralEndEffectorConstants.CORAL_WRIST_DEFAULT_POS;
        else requestedPosition = () -> CoralEndEffectorConstants.CORAL_HP_INTAKE_POS;
    }

    public double getWristPosition() {
        return coralWristInputs.unitPosition;
    }

    public boolean isOnTarget() {
        return ToleranceUtil.epsilonEquals(
            requestedPosition.getAsDouble(), 
            coralWristInputs.unitPosition, 
            CoralEndEffectorConstants.IS_ON_TARGET_THRESHOLD);
    }
}