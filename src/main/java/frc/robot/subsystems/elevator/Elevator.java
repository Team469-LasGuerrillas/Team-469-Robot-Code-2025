package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
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

    private int loopsSinceLastReset = 0;

    private Elevator(MotorIO coralElevatorMotor, MotorIO coralElevatorMotorFollower, MotorIO algaeElevatorMotor) {
        this.coralElevatorMotor = coralElevatorMotor;
        this.coralElevatorMotorFollower = coralElevatorMotorFollower;
        this.algaeElevatorMotor = algaeElevatorMotor;

        coralElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_CORAL_REST_POS_INCHES);
        algaeElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_ALGAE_REST_POS_INCHES);
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

        if (coralRequestedHeight.getAsDouble() - coralElevatorInputs.unitPosition > 0) coralElevatorMotor.setSlot(0);
        else coralElevatorMotor.setSlot(1);

        if (coralElevatorInputs.unitPosition < ElevatorConstants.CORAL_FEEDFORWARDS_HEIGHT_ZERO
            && coralRequestedHeight.getAsDouble() == ElevatorConstants.CORAL_DEFAULT_POS) appliedFF = ElevatorConstants.CORAL_FEEDFORWARDS_WHEN_ZERO;
        else if (coralElevatorInputs.unitPosition < ElevatorConstants.CORAL_FEEDFORWARDS_HEIGHT_L0) appliedFF = ElevatorConstants.CORAL_FEEDFORWARD_VOLTS_L0;
        else if (coralElevatorInputs.unitPosition < ElevatorConstants.CORAL_FEEDFORWARDS_HEIGHT_L1) appliedFF = ElevatorConstants.CORAL_FEEDFORWARD_VOLTS_L1;
        else appliedFF = ElevatorConstants.CORAL_FEEDFORWARD_VOLTS_L2;
        
        if (coralRequestedHeight.getAsDouble() > ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES) {
            updatedAlgaeRequestedHeight = algaeRequestedHeight.getAsDouble() - (coralRequestedHeight.getAsDouble() - ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES);
        }


        if (isAlgaeWristLegal() && isCoralWristLegal()) {
            double requestedVelocity = ElevatorConstants.CORAL_VELOCITY;
            if (coralElevatorInputs.unitPosition < ElevatorConstants.CORAL_SLOW_UPPER 
            && coralElevatorInputs.unitPosition > ElevatorConstants.CORAL_SLOW_LOWER 
            && coralElevatorInputs.velocityUnitsPerSecond < 0
            ) 
                requestedVelocity = ElevatorConstants.CORAL_SLOW_VELOCITY;            

            coralElevatorMotor.setDynamicMagicalPositionSetpoint(
                coralRequestedHeight.getAsDouble(), appliedFF, requestedVelocity, ElevatorConstants.CORAL_ACCELERATION, ElevatorConstants.CORAL_JERK
            );

            algaeElevatorMotor.setMagicalPositionSetpoint(updatedAlgaeRequestedHeight, ElevatorConstants.ALGAE_FEEDFORWARD_VOLTS);
        }

        // Automated Self-reset
        // Requires further testing
        // Logic:
        // If the requested position is at the bottom
        // AND the elevator velocity is at or near 0
        // AND the requested position hasn't been changed recently
        // AND a reset hasn't happened recently
        // THEN reset current position to the bottom
        if (coralRequestedHeight.getAsDouble() == ElevatorConstants.GROUND_TO_CORAL_REST_POS_INCHES &&
            algaeRequestedHeight.getAsDouble() == ElevatorConstants.GROUND_TO_ALGAE_REST_POS_INCHES &&
            DriverStation.isEnabled() &&
            Math.abs(coralElevatorInputs.velocityUnitsPerSecond) <= 0.05) // This number is just a guess (and can be moved to constants later)
        { 
            if (loopsSinceLastReset >= 25) // Conditions met AND have been met for consecutive loops. Trigger automated reset
            {
                loopsSinceLastReset = 0;
                coralElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_CORAL_REST_POS_INCHES);
                algaeElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_ALGAE_REST_POS_INCHES);
            }
            else // Conditions met BUT have not been met for consecurive loops. Increment counter.
                loopsSinceLastReset++;
            
        }
        else 
            loopsSinceLastReset = 0; // Conditions not met, reset loop counter
    }

    public void setTargetPosFromZero(DoubleSupplier targetCoralPosFromGroundInches, DoubleSupplier targetAlgaePosFromGroundInches) {
        boolean isPossibleTarget = isCarriageHeightsLegal(coralRequestedHeight.getAsDouble(), algaeRequestedHeight.getAsDouble());

        if (isPossibleTarget || true) { // JCAO: Should this || true still be in???
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
            && algaeRequestedHeight.getAsDouble() > ElevatorConstants.MIN_ELEVATOR_HEIGHT_FOR_ALGAE_OUT) ||
            (!GeomUtil.isLookingAtReef() 
            && !GeomUtil.isWithinReefRadius());

        boolean algaeUpCase =          AlgaeWristEndEffector.getInstance().getWristPosition() < AlgaeEndEffectorConstants.ALGAE_EXTENSION_THRESHOLD;

        return algaeOutCaseIsLegal || algaeUpCase;
    }

    private boolean isCoralWristLegal() {
        boolean coralIntakingCaseIsLegal =
            (CoralWristEndEffector.getInstance().getWristPosition() > CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_LOW
            && coralRequestedHeight.getAsDouble() < ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP_LOW);

        boolean coralOutCase = CoralWristEndEffector.getInstance().getWristPosition() > CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD_LOW;

        return coralIntakingCaseIsLegal || coralOutCase;
    }

    public double getCurrentCoralElevatorPosFromGroundInches() {
        return coralElevatorInputs.unitPosition;
    }

    public double getRequestedCoralElevatorPosFromGroundInches() {
        return coralRequestedHeight.getAsDouble();
    }

    public double getCurrentAlgaeElevatorPosFromGroundInches() {
        double coralCurrentPos = getCurrentCoralElevatorPosFromGroundInches();
        double algaePointOfReference = coralCurrentPos - ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES;

        return coralElevatorInputs.unitPosition > ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_GROUND_INCHES 
            ? algaeElevatorInputs.unitPosition + algaePointOfReference : algaeElevatorInputs.unitPosition;
    }

    public double getRequestedAlgaeElevatorPosFromGroundInches() {
        return algaeRequestedHeight.getAsDouble();
    }

    public boolean isCoralOnTarget() {
        boolean target = ToleranceUtil.epsilonEquals(
            coralRequestedHeight.getAsDouble(), coralElevatorInputs.unitPosition, ElevatorConstants.IS_ON_TARGET_THRESHOLD);

            System.out.println(target + " Setpoint: " + coralRequestedHeight.getAsDouble() + " Real: " + coralElevatorInputs.unitPosition);

        return target;
    }

    public boolean isAlgaeOnTarget() {
        return ToleranceUtil.epsilonEquals(
            algaeRequestedHeight.getAsDouble(), algaeElevatorInputs.unitPosition, ElevatorConstants.IS_ON_TARGET_THRESHOLD);
    }

    // True: Closer to Algae L3; False: Closer to Algae L2
    public boolean goToAlgaeL3() {
        ReefPositions closestReefPosition = FieldLayout.findClosestReefPoseRight();
        Pose2d closestPose = FieldLayout.reefPositionPoseRight.get(closestReefPosition);

        if (closestPose.getRotation().getDegrees() % 120 == 0) return true;
        return false;
    }

    public void resetElevatorState() {
        // TODO: Drive the stages down first!
        coralElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_CORAL_REST_POS_INCHES);
        algaeElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_ALGAE_REST_POS_INCHES);
    }

    // JCAO: DEAR GOD PLEASE KNOW WHAT YOU'RE DOING WHEN YOU USE THIS 
    // IT COULD BE BAD
    public void resetElevatorToHighState() {
        coralElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_CORAL_REST_POS_INCHES+24);
        algaeElevatorMotor.setCurrentPosition(ElevatorConstants.GROUND_TO_ALGAE_REST_POS_INCHES+24);
    }

    @AutoLogOutput
    public int getLoopsSinceLastElevatorReset() {
        return loopsSinceLastReset;
    }
}