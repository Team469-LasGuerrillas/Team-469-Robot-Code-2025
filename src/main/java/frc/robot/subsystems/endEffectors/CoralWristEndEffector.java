package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
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
        this.coralWristMotor = coralWristMotor;}

        public static CoralWristEndEffector createInstance(MotorIO coralWristMotor) {
            instance = new CoralWristEndEffector(coralWristMotor);
            return instance;
        }
    
        public static CoralWristEndEffector getInstance() {
            if (instance == null) {
                instance = new CoralWristEndEffector(new MotorIO() {});
            }
            return instance;
        
    }

    @Override
    public void periodic() {
        coralWristMotor.updateInputs(coralWristInputs);
        Logger.processInputs("Coral Wrist", coralWristInputs);

        if (isPositionAllowed(requestedPosition.getAsDouble())) {
            coralWristMotor.setMagicalPositionSetpoint(requestedPosition.getAsDouble(), 7);
        }
    }

    private boolean isPositionAllowed(double targetPosition) {
        return !(
            Elevator.getInstance().getCoralElevatorPosFromGroundInches() >= ElevatorConstants.MAX_ELEVATOR_HEIGHT_FOR_CORAL_FLIP
                && targetPosition < CoralEndEffectorConstants.CORAL_WRIST_FLIP_THRESHOLD
        );
    }

    public void setPosition(DoubleSupplier position) {
        requestedPosition = position;
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