package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.robot.subsystems.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {    
    private static Elevator instance;

    private final MotorIO coralElevatorMotor;
    private final MotorIOInputsAutoLogged coralElevatorInputs = new MotorIOInputsAutoLogged();

    private final MotorIO coralElevatorMotorFollower;
    private final MotorIOInputsAutoLogged coralElevatorFollowerInputs = new MotorIOInputsAutoLogged();

    private final MotorIO algaeElevatorMotor;
    private final MotorIOInputsAutoLogged algaeElevatorInputs = new MotorIOInputsAutoLogged();

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
    }

    public void setTargetPosFromZero(double targetCoralPosFromGround, double targetAlgaePosFromGround) {
        double algaeTargetHeight = targetAlgaePosFromGround - ElevatorConstants.GROUND_TO_ZERO_INCHES;
        double coralTargetHeight = targetCoralPosFromGround - ElevatorConstants.GROUND_TO_ZERO_INCHES;

        if (targetCoralPosFromGround - ElevatorConstants.GROUND_TO_ZERO_INCHES > ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES) {
            double currentCoralPosFromCoralZero = coralElevatorInputs.unitPosition;
            algaeTargetHeight = algaeTargetHeight - currentCoralPosFromCoralZero - ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES;
        }

        if (isPossibleTarget(targetCoralPosFromGround, targetAlgaePosFromGround)) {
            coralElevatorMotor.setMagicalPositionSetpoint(coralTargetHeight, 0);
            algaeElevatorMotor.setMagicalPositionSetpoint(algaeTargetHeight, 0);
        }
    }

    public void setAlgaePosFromZero(double targetAlgaePosFromGround) {
        double targetCoralPosFromGround = targetAlgaePosFromGround + ElevatorConstants.CARRIAGE_HEIGHT;
        setTargetPosFromZero(targetCoralPosFromGround, targetAlgaePosFromGround);
    }

    public void setCoralPosFromZero(double targetCoralPosFromGround) {
        double targetAlgaePosFromGround = targetCoralPosFromGround - ElevatorConstants.CARRIAGE_HEIGHT;
        setTargetPosFromZero(targetCoralPosFromGround, targetAlgaePosFromGround);
    }

    private boolean isPossibleTarget(double targetCoralPosFromGround, double targetAlgaePosFromGround) {
        if (targetCoralPosFromGround - targetAlgaePosFromGround <= ElevatorConstants.CARRIAGE_HEIGHT 
        || targetCoralPosFromGround - targetAlgaePosFromGround >= ElevatorConstants.MAX_CORAL_HEIGHT_IN_FIRST_STAGE_FROM_ZERO_INCHES)
            return false;
        return true;
    }
}