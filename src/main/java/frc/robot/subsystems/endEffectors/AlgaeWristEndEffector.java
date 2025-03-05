package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.SensorConstants;

public class AlgaeWristEndEffector extends SubsystemBase {
    private static AlgaeWristEndEffector instance;

    private final MotorIO algaeWristMotor;
    private final MotorIOInputsAutoLogged algaeWristInputs = new MotorIOInputsAutoLogged();

    private DoubleSupplier requestedPosition = () -> 0.23;

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

        algaeWristMotor.setMagicalPositionSetpoint(requestedPosition.getAsDouble(), -Math.cos((requestedPosition.getAsDouble() - AlgaeEndEffectorConstants.ALGAE_WRIST_HORZIONTAL_POS) * Math.PI * 2) * AlgaeEndEffectorConstants.VOLTAGE_TO_MAINTAIN_HORIZONTAL);
    }

    public void setPosition(DoubleSupplier position) {
        requestedPosition = position;
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

    public Command test() {
        return run(() -> setPosition(() -> 0.12));
    }
}

