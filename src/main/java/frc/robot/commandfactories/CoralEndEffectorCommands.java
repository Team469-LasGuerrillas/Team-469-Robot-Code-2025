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
        return Commands.either(
            Commands.run(() -> coralIntakeEndEffector.setVoltage(voltage), coralIntakeEndEffector), 
            Commands.none(), 
            () -> coralIntakeEndEffector.hasCoral()
        );
    }

    public static Command coralWrist(double position) {
        return Commands.run(() -> coralWristEndEffector.setPosition(position), coralWristEndEffector);
    }
}
