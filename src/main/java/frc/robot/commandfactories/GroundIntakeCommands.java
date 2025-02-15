package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.GroundIntakeConstants;
import frc.robot.subsystems.intake.GroundIntake;

public class GroundIntakeCommands {
    private static GroundIntake groundIntake = GroundIntake.getInstance();

    public static Command groundIntakeDown() {
        return Commands.run(() -> groundIntake.setPosition(GroundIntakeConstants.GROUND_INTAKE_DOWN), groundIntake);
    }

    public static Command groundIntakeUp() {
        return Commands.run(() -> groundIntake.setPosition(GroundIntakeConstants.GROUND_INTAKE_UP), groundIntake);
    }

    public static Command groundIntakeDefault() {
        return Commands.run(() -> groundIntake.setVoltage(GroundIntakeConstants.GROUND_DEFAULT_VOLTAGE), groundIntake);
    }

    public static Command groundIntakeIn() {
        return Commands.run(() -> groundIntake.setVoltage(GroundIntakeConstants.GROUND_INTAKE_IN_VOLTAGE), groundIntake);
    }

    public static Command groundIntakeOut() {
        return Commands.run(() -> groundIntake.setVoltage(GroundIntakeConstants.GROUND_INTAKE_OUT_VOLTAGE), groundIntake);
    }
}