package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.endEffectors.AlgaeIntakeEndEffector;
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;

public class AlgaeEndEffectorCommands {
    private static AlgaeIntakeEndEffector algaeIntakeEndEffector = AlgaeIntakeEndEffector.getInstance();
    private static AlgaeWristEndEffector algaeWristEndEffector = AlgaeWristEndEffector.getInstance();

     public static Command algaeIntake(DoubleSupplier voltage) {
        return Commands.either( 
            Commands.none(),
            Commands.run(() -> algaeIntakeEndEffector.setVoltage(voltage), algaeIntakeEndEffector),
            () -> algaeIntakeEndEffector.hasAlgae()
        );
    }

    public static Command algaeWrist(DoubleSupplier position) {
        return Commands.run(() -> algaeWristEndEffector.setPosition(position), algaeWristEndEffector);
    }
}