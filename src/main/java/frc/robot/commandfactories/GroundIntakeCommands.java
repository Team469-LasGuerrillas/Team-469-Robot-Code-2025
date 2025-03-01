package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.GroundIntakeConstants;
import frc.robot.subsystems.intake.GroundIntake;
import frc.robot.subsystems.intake.GroundWrist;

public class GroundIntakeCommands {
    private static GroundIntake groundIntake = GroundIntake.getInstance();
    private static GroundWrist groundWrist = GroundWrist.getInstance();

    public static Command groundIntake(DoubleSupplier voltage) {
        return Commands.deferredProxy(
            () -> Commands.either(
            Commands.none(),
            Commands.run(() -> groundIntake.setVoltage(voltage), groundIntake),
            () -> groundIntake.hasCoral()));
        
    }

    public static Command groundWrist(double Position) {
        return Commands.run(() -> groundWrist.setPosition(Position), groundWrist);
    }
}