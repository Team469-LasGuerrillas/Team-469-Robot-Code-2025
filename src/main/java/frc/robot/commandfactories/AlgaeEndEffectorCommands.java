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
        return Commands.deferredProxy(
            () -> Commands.either(
            Commands.none(),
            runAlgaeIntake(voltage),
            () -> algaeIntakeEndEffector.hasAlgae()
        ));
    }

    public static Command runAlgaeIntake(DoubleSupplier voltage) {
        return Commands.startRun(() -> algaeIntakeEndEffector.setVoltage(voltage), () -> algaeIntakeEndEffector.setVoltage(voltage), algaeIntakeEndEffector);
    }

    public static Command algaeWrist(DoubleSupplier position) {
        return Commands.startRun(() -> algaeWristEndEffector.setPosition(position), () -> algaeWristEndEffector.setPosition(position), algaeWristEndEffector);
    }

    public static Command algaeWrist() {
        return Commands.deferredProxy(
            () -> Commands.either(
            Commands.none(),
            Commands.run(() -> algaeWristEndEffector.setPosition(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_L2_L3), algaeWristEndEffector),
            () -> algaeIntakeEndEffector.isSeeingAlgae()
            )
        );
    }
}