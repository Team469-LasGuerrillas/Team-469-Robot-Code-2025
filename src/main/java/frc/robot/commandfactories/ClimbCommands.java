package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.ClimbConstants;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommands {
    private static Climb climb = Climb.getInstance();

    public static Command climbExtend() {
        return Commands.run(() -> climb.setVoltage(ClimbConstants.CLIMB_EXTEND), climb);
    }

    public static Command climbRetract() {
        return Commands.run(() -> climb.setVoltage(ClimbConstants.CLIMB_RETRACT), climb);
    }
}