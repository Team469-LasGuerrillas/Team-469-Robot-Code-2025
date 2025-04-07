package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.AutoScore;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.endEffectors.AlgaeIntakeEndEffector;
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;

public class AlgaeEndEffectorCommands {
    private static AlgaeIntakeEndEffector algaeIntakeEndEffector = AlgaeIntakeEndEffector.getInstance();
    private static AlgaeWristEndEffector algaeWristEndEffector = AlgaeWristEndEffector.getInstance();

    public static Command algaeIntake(DoubleSupplier voltage) {
        // return Commands.startRun(() -> algaeIntakeEndEffector.setVoltage(()->0), () -> algaeIntakeEndEffector.setVoltage(()->0), algaeIntakeEndEffector);
        return Commands.startRun(() -> algaeIntakeEndEffector.setVoltage(voltage), () -> algaeIntakeEndEffector.setVoltage(voltage), algaeIntakeEndEffector);
    }

    public static Command algaeDynamicIntakeOut() {
        return Commands.startRun(() -> algaeIntakeEndEffector.setDynamicOutVoltage(), () -> algaeIntakeEndEffector.setDynamicOutVoltage(), algaeIntakeEndEffector);
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

    public static Command algaeWristDefault() {
        return Commands.startRun(
            () -> AlgaeWristEndEffector.getInstance().setDefault(),
            () -> AlgaeWristEndEffector.getInstance().setDefault(),
            algaeWristEndEffector
        );
    }

    public static Command algaeWristAutoScore(Double nextPos) {
        return Commands.run(() -> AutoScore.setNextAlgaeWristPos(nextPos), new Subsystem[]{});
    }

    public static Command algaeIntakeAutoScore(Double nextVol) {
        return Commands.run(() -> AutoScore.setNextAlgaeIntakeVol(nextVol), new Subsystem[]{});
    }
}