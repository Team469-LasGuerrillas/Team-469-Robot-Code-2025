package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.ReefPositions;
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

    private DoubleSupplier coralRequestedHeight = () -> ElevatorConstants.CORAL_DEFAULT_POS;
    private DoubleSupplier algaeRequestedHeight = () -> ElevatorConstants.ALGAE_DEFAULT_POS;

    private Elevator(MotorIO coralElevatorMotor, MotorIO coralElevatorMotorFollower, MotorIO algaeElevatorMotor) {
        this.coralElevatorMotor = coralElevatorMotor;
        this.coralElevatorMotorFollower = coralElevatorMotorFollower;
        this.algaeElevatorMotor = algaeElevatorMotor;

        coralElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_CORAL_REST_POS_INCHES);
    }

    public static Elevator createInstance(MotorIO coralElevatorMotor, MotorIO coralElevatorMotorFollower, MotorIO algaeElevatorMotor) {
        instance = new Elevator(coralElevatorMotor, coralElevatorMotorFollower, algaeElevatorMotor);
        return instance;
    }

    public static Elevator getInstance() {
        if (instance == null) throw new Error("Subsystem has not been created");
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

        double updatedAlgaeRequestedHeight = algaeRequestedHeight.getAsDouble();

        double appliedFF;

        if (coralElevatorInputs.unitPosition < 17.63) appliedFF = ElevatorConstants.CORAL_FEEDFORWARD_VOLTS_L0;
        else if (coralElevatorInputs.unitPosition < 23.72) appliedFF = ElevatorConstants.CORAL_FEEDFORWARD_VOLTS_L1;
        else appliedFF = ElevatorConstants.CORAL_FEEDFORWARD_VOLTS_L2;
        
        if (coralRequestedHeight.getAsDouble() > ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES) {
            updatedAlgaeRequestedHeight = algaeRequestedHeight.getAsDouble() - (coralRequestedHeight.getAsDouble() + ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES);
        }

        if (isAlgaeWristLegal() && isCoralWristLegal() || true) {
            coralElevatorMotor.setMagicalPositionSetpoint(coralRequestedHeight.getAsDouble(), appliedFF);
            algaeElevatorMotor.setMagicalPositionSetpoint(updatedAlgaeRequestedHeight, ElevatorConstants.ALGAE_FEEDFORWARD_VOLTS);
        }
    }

    public void setTargetPosFromZero(DoubleSupplier targetCoralPosFromGroundInches, DoubleSupplier targetAlgaePosFromGroundInches) {
        boolean isPossibleTarget = isCarriageHeightsLegal(coralRequestedHeight.getAsDouble(), algaeRequestedHeight.getAsDouble());

        if (isPossibleTarget || true) {
            coralRequestedHeight = targetCoralPosFromGroundInches;
            algaeRequestedHeight = targetAlgaePosFromGroundInches;
        }
    }

    public void setAlgaePosFromZero(DoubleSupplier targetAlgaePosFromGroundInches) {
        double targetCoralPosFromGround = targetAlgaePosFromGroundInches.getAsDouble() + ElevatorConstants.CARRIAGE_HEIGHT;
        setTargetPosFromZero(() -> targetCoralPosFromGround, targetAlgaePosFromGroundInches);
    }

    public void setCoralPosFromZero(DoubleSupplier targetCoralPosFromGroundInches) {
        double targetAlgaePosFromGround = targetCoralPosFromGroundInches.getAsDouble() - ElevatorConstants.CARRIAGE_HEIGHT;
        setTargetPosFromZero(targetCoralPosFromGroundInches, () -> targetAlgaePosFromGround);
    }

    private boolean isCarriageHeightsLegal(double targetCoralPosFromGroundInches, double targetAlgaePosFromGroundInches) {
        if (targetCoralPosFromGroundInches - targetAlgaePosFromGroundInches <= ElevatorConstants.CARRIAGE_HEIGHT // If algae carriage is above or on top of coral carriage
        || targetAlgaePosFromGroundInches < ElevatorConstants.GROUND_TO_ALGAE_REST_POS_INCHES // If algae carriage is trying to go below 0 / the first stage
        || targetCoralPosFromGroundInches - targetAlgaePosFromGroundInches >= ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES) // If algae carriage and coral carriage are too far apart from each other
            return false;
        return true;
    }

    private boolean isAlgaeWristLegal() {
        boolean algaeOutCaseIsLegal = (AlgaeWristEndEffector.getInstance().getWristPosition() > AlgaeEndEffectorConstants.ALGAE_EXTENSION_THRESHOLD
            && algaeRequestedHeight.getAsDouble() > ElevatorConstants.MIN_ELEVATOR_HEIGHT_FOR_ALGAE_OUT)
            && GeomUtil.isLookingAtReef() 
            && GeomUtil.isWithinReefRadius();
        boolean algaeUpCase =          AlgaeWristEndEffector.getInstance().getWristPosition() < AlgaeEndEffectorConstants.ALGAE_EXTENSION_THRESHOLD;

        return algaeOutCaseIsLegal || algaeUpCase;
    }

    private boolean isCoralWristLegal() {
        boolean coralIntakingCaseIsLegal = (CoralWristEndEffector.getInstance().getWristPosition() < CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD
            && coralRequestedHeight.getAsDouble() < ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP);
        boolean coralOutCase =              CoralWristEndEffector.getInstance().getWristPosition() > CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD;

        return coralIntakingCaseIsLegal || coralOutCase;
    }

    public double getCoralElevatorPosFromGroundInches() {
        return coralElevatorInputs.unitPosition;
    }

    public double getAlgaeElevatorPosFromGroundInches() {
        double coralCurrentPos = getCoralElevatorPosFromGroundInches();
        double algaePointOfReference = coralCurrentPos - ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES;

        return coralElevatorInputs.unitPosition > ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES 
            ? algaeElevatorInputs.unitPosition + algaePointOfReference : algaeElevatorInputs.unitPosition;
    }

    public boolean isOnTarget() {
        boolean isOnTargetCoral = ToleranceUtil.epsilonEquals(
            coralRequestedHeight.getAsDouble(), coralElevatorInputs.unitPosition, ElevatorConstants.IS_ON_TARGET_THRESHOLD);
        
        boolean isOnTargetAlgae = ToleranceUtil.epsilonEquals(
            algaeRequestedHeight.getAsDouble(), algaeElevatorInputs.unitPosition, ElevatorConstants.IS_ON_TARGET_THRESHOLD);

        return isOnTargetCoral && isOnTargetAlgae;
    }

    // True: Closer to Algae L3; False: Closer to Algae L2
    public boolean goToAlgaeL3() {
        ReefPositions closestReefPosition = FieldLayout.findClosestReefPoseRight();
        Pose2d closestPose = FieldLayout.reefPositionPoseRight.get(closestReefPosition);

        if (closestPose.getRotation().getDegrees() % 120 == 0) return true;
        return false;
    }
}