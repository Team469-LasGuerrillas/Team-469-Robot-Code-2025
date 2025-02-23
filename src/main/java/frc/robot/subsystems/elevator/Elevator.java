package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.util.math.GeomUtil;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;
import frc.robot.subsystems.endEffectors.CoralWristEndEffector;

public class Elevator extends SubsystemBase {    
    private static Elevator instance;

    private final MotorIO coralElevatorMotor;
    private final MotorIOInputsAutoLogged coralElevatorInputs = new MotorIOInputsAutoLogged();

    private final MotorIO coralElevatorMotorFollower;
    private final MotorIOInputsAutoLogged coralElevatorFollowerInputs = new MotorIOInputsAutoLogged();

    private final MotorIO algaeElevatorMotor;
    private final MotorIOInputsAutoLogged algaeElevatorInputs = new MotorIOInputsAutoLogged();


    private double coralRequestedHeight = ElevatorConstants.CORAL_RESTING_POS;
    private double algaeRequestedHeight = ElevatorConstants.ALGAE_RESTING_POS;

    private Elevator(MotorIO coralElevatorMotor, MotorIO coralElevatorMotorFollower, MotorIO algaeElevatorMotor) {
        this.coralElevatorMotor = coralElevatorMotor;
        this.coralElevatorMotorFollower = coralElevatorMotorFollower;
        this.algaeElevatorMotor = algaeElevatorMotor;
    }

    public static Elevator createInstance(MotorIO coralElevatorMotor, MotorIO coralElevatorMotorFollower, MotorIO algaeElevatorMotor) {
        instance = new Elevator(coralElevatorMotor, coralElevatorMotorFollower, algaeElevatorMotor);
        return instance;
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator(new MotorIO() {}, new MotorIO() {}, new MotorIO() {});
        }
        return instance;
    }

    @Override
    public void periodic() {
        coralElevatorMotor.updateInputs(coralElevatorInputs);
        Logger.processInputs("coralElevator", coralElevatorInputs);

        coralElevatorMotorFollower.updateInputs(coralElevatorFollowerInputs);
        Logger.processInputs("coralElevatorFollower", coralElevatorFollowerInputs);

        algaeElevatorMotor.updateInputs(algaeElevatorInputs);
        Logger.processInputs("algaeElevator", algaeElevatorInputs);

        if (coralRequestedHeight > ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES) {
            algaeRequestedHeight -= (coralRequestedHeight + ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES);
        }

        if (isAlgaeWristLegal() && isCoralWristLegal()) {
            coralElevatorMotor.setMagicalPositionSetpoint(coralRequestedHeight, ElevatorConstants.FEEDFORWARD_VOLTS);
            algaeElevatorMotor.setMagicalPositionSetpoint(algaeRequestedHeight, ElevatorConstants.FEEDFORWARD_VOLTS);
        }
    }

    public void setTargetPosFromZero(double targetCoralPosFromGroundInches, double targetAlgaePosFromGroundInches) {
        boolean isPossibleTarget = isCarriageHeightsLegal(coralRequestedHeight, algaeRequestedHeight);

        if (isPossibleTarget) {
            coralRequestedHeight = targetCoralPosFromGroundInches;
            algaeRequestedHeight = targetAlgaePosFromGroundInches;
        }
    }

    public void setAlgaePosFromZero(double targetAlgaePosFromGroundInches) {
        double targetCoralPosFromGround = targetAlgaePosFromGroundInches + ElevatorConstants.CARRIAGE_HEIGHT;
        setTargetPosFromZero(targetCoralPosFromGround, targetAlgaePosFromGroundInches);
    }

    public void setCoralPosFromZero(double targetCoralPosFromGroundInches) {
        double targetAlgaePosFromGround = targetCoralPosFromGroundInches - ElevatorConstants.CARRIAGE_HEIGHT;
        setTargetPosFromZero(targetCoralPosFromGroundInches, targetAlgaePosFromGround);
    }

    private boolean isCarriageHeightsLegal(double targetCoralPosFromGroundInches, double targetAlgaePosFromGroundInches) {
        if (targetCoralPosFromGroundInches - targetAlgaePosFromGroundInches <= ElevatorConstants.CARRIAGE_HEIGHT // If algae carriage is above or on top of coral carriage
        || targetAlgaePosFromGroundInches < ElevatorConstants.GROUND_TO_ALGAE_REST_POS_INCHES // If algae carriage is trying to go below 0 / the first stage
        || targetCoralPosFromGroundInches - targetAlgaePosFromGroundInches >= ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES) // If algae carriage and coral carriage are too far apart from each other
            return false;
        return true;
    }

    private boolean isAlgaeWristLegal() {
        boolean algaeOutCaseIsLegal = (AlgaeWristEndEffector.getInstance().getWristPosition() > AlgaeEndEffectorConstants.ALGAE_EXTENSION_THRESHOLD
            && algaeRequestedHeight > ElevatorConstants.MIN_ELEVATOR_HEIGHT_FOR_ALGAE_OUT)
            && GeomUtil.isLookingAtReef() 
            && GeomUtil.isWithinReefRadius();
        boolean algaeUpCase =          AlgaeWristEndEffector.getInstance().getWristPosition() < AlgaeEndEffectorConstants.ALGAE_EXTENSION_THRESHOLD;

        return algaeOutCaseIsLegal || algaeUpCase;
    }

    private boolean isCoralWristLegal() {
        boolean coralIntakingCaseIsLegal = (CoralWristEndEffector.getInstance().getWristPosition() < CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD
            && coralRequestedHeight < ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP);
        boolean coralOutCase =              CoralWristEndEffector.getInstance().getWristPosition() > CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD;

        return coralIntakingCaseIsLegal || coralOutCase;
    }

    public double getCoralElevatorPosFromGroundInches() {
        return coralElevatorInputs.unitPosition;
    }

    public double getAlgaeElevatorPosFromGroundInches() {
        double coralCurrentPos = getCoralElevatorPosFromGroundInches();
        double algaePointOfReference = coralCurrentPos - ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES;

        return coralElevatorInputs.unitPosition > ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES 
            ? algaeElevatorInputs.unitPosition + algaePointOfReference : algaeElevatorInputs.unitPosition;
    }

    public boolean isOnTarget() {
        boolean isOnTargetCoral = ToleranceUtil.epsilonEquals(
            coralRequestedHeight, coralElevatorInputs.unitPosition, ElevatorConstants.IS_ON_TARGET_THRESHOLD);
        
        boolean isOnTargetAlgae = ToleranceUtil.epsilonEquals(
            algaeRequestedHeight, algaeElevatorInputs.unitPosition, ElevatorConstants.IS_ON_TARGET_THRESHOLD);

        return isOnTargetCoral && isOnTargetAlgae;
    }
}