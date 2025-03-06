package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.endEffectors.CoralIntakeEndEffector;
import frc.robot.subsystems.endEffectors.CoralWristEndEffector;

public class CoralEndEffectorCommands {
    private static CoralIntakeEndEffector coralIntakeEndEffector = CoralIntakeEndEffector.getInstance();
    private static CoralWristEndEffector coralWristEndEffector = CoralWristEndEffector.getInstance();

    public static Command coralIntake(DoubleSupplier voltage) {
        return Commands.deferredProxy(
            () -> Commands.either(
            runCoralIntake(voltage),
            Commands.none(),
            () -> 
            // coralIntakeEndEffector.hasCoral() ||
             true
            ));
    }

    public static Command runCoralIntake(DoubleSupplier voltage) {
        return Commands.startRun(() -> coralIntakeEndEffector.setVoltage(voltage), () -> coralIntakeEndEffector.setVoltage(voltage), coralIntakeEndEffector);
    }


    public static Command coralWrist(DoubleSupplier position) {
        return Commands.startRun(() -> coralWristEndEffector.setPosition(position), () -> coralWristEndEffector.setPosition(position), coralWristEndEffector);
    }
}
