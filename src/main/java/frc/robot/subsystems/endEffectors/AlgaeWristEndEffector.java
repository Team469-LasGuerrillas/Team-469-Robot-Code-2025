package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.util.math.GeomUtil;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.constants.SensorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class AlgaeWristEndEffector extends SubsystemBase {
    private static AlgaeWristEndEffector instance;

    private final MotorIO algaeWristMotor;
    private final MotorIOInputsAutoLogged algaeWristInputs = new MotorIOInputsAutoLogged();

    private DoubleSupplier requestedPosition = () -> AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS;

    private AlgaeWristEndEffector(MotorIO algaeWristMotor) {
        this.algaeWristMotor = algaeWristMotor;
    }

    public static AlgaeWristEndEffector createInstance(MotorIO algaeWristMotor) {
        instance = new AlgaeWristEndEffector(algaeWristMotor);
        return instance;
    }

    public static AlgaeWristEndEffector getInstance() {
        if (instance == null) throw new Error("Subsystem has not been created");
        return instance;
    }

    @Override
    public void periodic() {
        algaeWristMotor.updateInputs(algaeWristInputs);
        Logger.processInputs("Algae Wrist", algaeWristInputs);
    
        // TODO: UMMMMM SHOULDN'T THIS BE REQUESTED POSITION??????
        if (isPositionAllowed(getWristPosition()))
            algaeWristMotor.setMagicalPositionSetpoint(requestedPosition.getAsDouble(), -Math.cos((requestedPosition.getAsDouble() - AlgaeEndEffectorConstants.ALGAE_WRIST_HORZIONTAL_POS) * Math.PI * 2) * AlgaeEndEffectorConstants.VOLTAGE_TO_MAINTAIN_HORIZONTAL);
    }

    private boolean isPositionAllowed(double position) {
        return 
            (GeomUtil.isLookingAtReef() 
            && GeomUtil.isWithinReefRadius() 
            && Elevator.getInstance().getCurrentAlgaeElevatorPosFromGroundInches() >= ElevatorConstants.MIN_ELEVATOR_HEIGHT_FOR_ALGAE_OUT
            && Elevator.getInstance().getRequestedAlgaeElevatorPosFromGroundInches() >= ElevatorConstants.MIN_ELEVATOR_HEIGHT_FOR_ALGAE_OUT)
            || (Elevator.getInstance().getRequestedAlgaeElevatorPosFromGroundInches() == ElevatorConstants.ALGAE_GROUND_POS)
            || (Elevator.getInstance().getRequestedAlgaeElevatorPosFromGroundInches() == ElevatorConstants.ALGAE_PROCESSOR_POS)
            || (Elevator.getInstance().getRequestedAlgaeElevatorPosFromGroundInches() == ElevatorConstants.ALGAE_DEFAULT_POS)
            || (!GeomUtil.isLookingAtReef() || !GeomUtil.isWithinReefRadius());
    }

    public void setPosition(DoubleSupplier position) {
        requestedPosition = position;
    }

    @AutoLogOutput
    public double getRequestedPosition() {
        return requestedPosition.getAsDouble();
    }

    public void setDefault() {
        if (AlgaeIntakeEndEffector.getInstance().hasAlgae())
            requestedPosition = () -> AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS_WA;
        else requestedPosition = () -> AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS;
    }

    public double getWristPosition() {
        return algaeWristInputs.unitPosition;
    }

    public boolean isOnTarget() {
        return ToleranceUtil.epsilonEquals(
            requestedPosition.getAsDouble(), 
            algaeWristInputs.unitPosition, 
            AlgaeEndEffectorConstants.IS_ON_TARGET_THRESHOLD);
    }
}

