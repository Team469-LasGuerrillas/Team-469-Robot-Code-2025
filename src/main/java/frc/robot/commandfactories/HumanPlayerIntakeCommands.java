package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.HumanPlayerIntakeConstants;
import frc.robot.subsystems.intake.HumanPlayerIntake;

public class HumanPlayerIntakeCommands {
    private static HumanPlayerIntake humanPlayerIntake = HumanPlayerIntake.getInstance();

    public static Command hpIntakeIn() {
        return Commands.run(() -> humanPlayerIntake.setVoltage(HumanPlayerIntakeConstants.HP_INTAKE_IN), humanPlayerIntake);
    }

    public static Command hpIntakeOut() {
        return Commands.run(() -> humanPlayerIntake.setVoltage(HumanPlayerIntakeConstants.HP_INTAKE_OUT), humanPlayerIntake);
    }

    public static Command hpIntakeDefault() {
        return Commands.run(() -> humanPlayerIntake.setVoltage(HumanPlayerIntakeConstants.HP_INTAKE_DEFAULT), humanPlayerIntake);
    }
}