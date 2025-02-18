package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

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

    public void setCoralHeightFromGround(double heightInInchesFromGround) {
        double currentCoralHeightFromZero = coralElevatorInputs.unitPosition * ElevatorConstants.CORAL_STAGE_HEIGHT_FACTOR;
        double coralStageHeight = 
        heightInInchesFromGround
        -  currentCoralHeightFromZero
        - ElevatorConstants.GROUND_TO_CORAL_REST_POS_INCHES;

        double rotations = coralStageHeight / ElevatorConstants.CORAL_STAGE_HEIGHT_FACTOR;

        coralElevatorMotor.setMagicalPositionSetpoint(rotations, 0);     
    }

    public void setAlgaeHeightFromGround(double heightInInchesFromGround) {
        double currentAlgaeHeightFromZero = algaeElevatorInputs.unitPosition * ElevatorConstants.ALGAE_STAGE_HEIGHT_FACTOR;
        double currentCoralHeightFromZero = coralElevatorInputs.unitPosition * ElevatorConstants.CORAL_STAGE_HEIGHT_FACTOR;

        double algaeStageHeight;
        double coralZeroToAlgaeZero = ElevatorConstants.CORAL_CARRIAGE_TO_ALGAE_ZERO;

        if (currentCoralHeightFromZero < ElevatorConstants.ALGAE_RELATIVE_TO_CORAL_HEIGHT) {
            algaeStageHeight = 
            heightInInchesFromGround
            - currentAlgaeHeightFromZero
            - ElevatorConstants.GROUND_TO_ALGAE_REST_POS_INCHES;
        }
        else {
            coralZeroToAlgaeZero = currentCoralHeightFromZero - coralZeroToAlgaeZero;

            algaeStageHeight = 
            heightInInchesFromGround
            - coralZeroToAlgaeZero
            - currentAlgaeHeightFromZero
            - ElevatorConstants.GROUND_TO_CORAL_REST_POS_INCHES;
        }

        if (currentAlgaeHeightFromZero + algaeStageHeight > 0 
        && coralZeroToAlgaeZero
         + currentAlgaeHeightFromZero
          + ElevatorConstants.ALGAE_CARRIAGE_HEIGHT
           + algaeStageHeight < currentCoralHeightFromZero) {
            double rotations = algaeStageHeight / ElevatorConstants.CORAL_STAGE_HEIGHT_FACTOR;
            algaeElevatorMotor.setMagicalPositionSetpoint(rotations, 0);
        }
    }
}