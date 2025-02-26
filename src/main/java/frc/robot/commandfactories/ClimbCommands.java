package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.ClimbConstants;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommands {
    private static Climb climb = Climb.getInstance();

    public static Command climb(DoubleSupplier voltage) {
        return Commands.run(() -> climb.setVoltage(voltage), climb);
    }
}