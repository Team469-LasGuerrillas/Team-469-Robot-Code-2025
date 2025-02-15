package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.endEffectors.CoralEndEffector;

public class CoralEndEffectorCommands {
    private static CoralEndEffector coralEndEffector = CoralEndEffector.getInstance();

    public static Command coralIntakeIn() {
        return Commands.run(() -> coralEndEffector.setVoltage(CoralEndEffectorConstants.CORAL_INTAKE_IN_VOLTAGE), coralEndEffector);
    }

    public static Command coralIntakeOut() {
        return Commands.run(() -> coralEndEffector.setVoltage(CoralEndEffectorConstants.CORAL_INTAKE_OUT_VOLTAGE), coralEndEffector);
    }

    public static Command coralIntakeDefault() {
        return Commands.run(() -> coralEndEffector.setVoltage(CoralEndEffectorConstants.CORAL_DEFAULT_VOLTAGE), coralEndEffector);
    }

    public static Command coralWristL4() {
        return Commands.run(() -> coralEndEffector.setPosition(CoralEndEffectorConstants.CORAL_L4_POSITION), coralEndEffector);
    }

    public static Command coralWristL3() {
        return Commands.run(() -> coralEndEffector.setPosition(CoralEndEffectorConstants.CORAL_L3_POSITION), coralEndEffector);
    }

    public static Command coralWristL2() {
        return Commands.run(() -> coralEndEffector.setPosition(CoralEndEffectorConstants.CORAL_L2_POSITION), coralEndEffector);
    }

    public static Command coralWristL1() {
        return Commands.run(() -> coralEndEffector.setPosition(CoralEndEffectorConstants.CORAL_L1_POSITION), coralEndEffector);
    }

    public static Command coralWristGroundIntake() {
        return Commands.run(() -> coralEndEffector.setPosition(CoralEndEffectorConstants.CORAL_GROUND_INTAKE_POSITION), coralEndEffector);
    }

    public static Command coralWristHPIntake() {
        return Commands.run(() -> coralEndEffector.setPosition(CoralEndEffectorConstants.CORAL_HP_INTAKE_POSITION), coralEndEffector);
    }
}
