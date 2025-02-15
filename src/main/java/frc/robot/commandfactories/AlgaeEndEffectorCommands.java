package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.endEffectors.AlgaeEndEffector;

public class AlgaeEndEffectorCommands {
    private static AlgaeEndEffector algaeEndEffector = AlgaeEndEffector.getInstance();

    public static Command algaeIntakeIn() {
        return Commands.run(() -> algaeEndEffector.setVoltage(AlgaeEndEffectorConstants.ALGAE_INTAKE_IN_VOLTAGE), algaeEndEffector);
    }

    public static Command algaeIntakeOut() {
        return Commands.run(() -> algaeEndEffector.setVoltage(AlgaeEndEffectorConstants.ALGAE_INTAKE_OUT_VOLAGE), algaeEndEffector);
    }

    public static Command stopAlgaeIntake() {
        return Commands.run(() -> algaeEndEffector.setVoltage(0), algaeEndEffector);
    }

    public static Command algaeWristDefault() {
        return Commands.run(() -> algaeEndEffector.setPosition(AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT), algaeEndEffector);
    }

    public static Command algaeWristProcessor() {
        return Commands.run(() -> algaeEndEffector.setPosition(AlgaeEndEffectorConstants.ALGAE_WRIST_PROCESSOR), algaeEndEffector);
    }

    public static Command algaeWristBarge() {
        return Commands.run(() -> algaeEndEffector.setPosition(AlgaeEndEffectorConstants.ALGAE_WRIST_BARGE), algaeEndEffector);
    }

    public static Command algaeWristGround() {
        return Commands.run(() -> algaeEndEffector.setPosition(AlgaeEndEffectorConstants.ALGAE_WRIST_GROUND), algaeEndEffector);
    }

    public static Command algaeWristReef() {
        return Commands.run(() -> algaeEndEffector.setPosition(AlgaeEndEffectorConstants.ALGAE_WRIST_REEF), algaeEndEffector);
    }
}
