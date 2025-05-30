package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.util.AutoScore;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class CoralWristEndEffector extends SubsystemBase {
    private static CoralWristEndEffector instance;  

    private final MotorIO coralWristMotor;
    MotorIOInputsAutoLogged coralWristInputs = new MotorIOInputsAutoLogged();

    private DoubleSupplier requestedPosition = () -> CoralEndEffectorConstants.CORAL_WRIST_DEFAULT_POS;

    int isOnTargetLoopCount = 0;

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

        if (isCoralWristOnTarget()) isOnTargetLoopCount++;
        else isOnTargetLoopCount = 0;

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
        if ((Elevator.getInstance().getCurrentCoralElevatorPosFromGroundInches() <= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_IDLE
            || Elevator.getInstance().getRequestedCoralElevatorPosFromGroundInches() <= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_IDLE)
            && targetPosition > CoralEndEffectorConstants.IDLE_WRIST_THRESHOLD) {
            if (targetPosition == CoralEndEffectorConstants.CORAL_L1_POS) 
                return CoralEndEffectorConstants.IDLE_WRIST_THRESHOLD_2;
            return CoralEndEffectorConstants.IDLE_WRIST_THRESHOLD;
        } else if ((Elevator.getInstance().getCurrentCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_HIGH
            || Elevator.getInstance().getRequestedCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_HIGH)
            && targetPosition < CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_HIGH) {
            return CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_HIGH;
        } else if ((Elevator.getInstance().getCurrentCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_LOW_WITHOUT_CORAL
            || Elevator.getInstance().getRequestedCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_LOW_WITHOUT_CORAL)
            && !CoralIntakeEndEffector.getInstance().hasCoral()
            && targetPosition < CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_LOW) {
            return CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_LOW;
        } else if ((Elevator.getInstance().getCurrentCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_LOW_WITH_CORAL
            || Elevator.getInstance().getRequestedCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_LOW_WITH_CORAL)
            && CoralIntakeEndEffector.getInstance().hasCoral()
            && targetPosition < CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_LOW) {
            return CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_LOW;
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

    @AutoLogOutput
    public boolean isCoralWristOnTarget() {
        return ToleranceUtil.epsilonEquals(
            requestedPosition.getAsDouble(), 
            coralWristInputs.unitPosition, 
            CoralEndEffectorConstants.IS_ON_TARGET_THRESHOLD);
    }

    public boolean isCoralWristOnTarget(int numOfOnTargetLoops) {
        return isOnTargetLoopCount > numOfOnTargetLoops;
    }

    @AutoLogOutput
    public double getNextCoralWristPosition() {
        return AutoScore.getNextCoralWristPos().getAsDouble();
    }

    @AutoLogOutput
    public double getRequestedPosition() {
        return requestedPosition.getAsDouble();
    }
}