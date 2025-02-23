package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.HumanPlayerIntakeConstants;
import frc.robot.subsystems.intake.HumanPlayerIntake;

public class HumanPlayerIntakeCommands {
    private static HumanPlayerIntake humanPlayerIntake = HumanPlayerIntake.getInstance();

    public static Command humanPlayerIntake(DoubleSupplier voltage) {
        return Commands.either(
            Commands.none(), 
            Commands.run(() -> humanPlayerIntake.setVoltage(voltage), humanPlayerIntake), 
            () -> humanPlayerIntake.hasCoral()
        );
    }
}