package frc.robot.subsystems.endEffectors;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOInputsAutoLogged;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;

public class AlgaeWristEndEffector extends SubsystemBase {
    private static AlgaeWristEndEffector instance;

    private final MotorIO algaeWristMotor;
    private final MotorIOInputsAutoLogged algaeWristInputs = new MotorIOInputsAutoLogged();

    private DoubleSupplier requestedPosition = () -> AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT;

    private AlgaeWristEndEffector(MotorIO algaeWristMotor) {
        this.algaeWristMotor = algaeWristMotor;
    }

    public static AlgaeWristEndEffector createInstance(MotorIO algaeWristMotor) {
        instance = new AlgaeWristEndEffector(algaeWristMotor);
        return instance;
    }

    public static AlgaeWristEndEffector getInstance() {
        if (instance == null) {
            instance = new AlgaeWristEndEffector(new MotorIO() {});
        }
        return instance;
    }

    @Override
    public void periodic() {
        algaeWristMotor.updateInputs(algaeWristInputs);
        Logger.processInputs("Algae Wrist", algaeWristInputs);

        algaeWristMotor.setMagicalPositionSetpoint(requestedPosition.getAsDouble(), AlgaeEndEffectorConstants.ALGAE_WRIST_FEED_FORWARD_VOLTS);
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
}

