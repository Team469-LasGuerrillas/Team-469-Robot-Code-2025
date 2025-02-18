package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {
    private static Elevator elevator = Elevator.getInstance();

    public static Command coralExtend() {
        return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_STAGE_UP), elevator);
    }

    public static Command coralRetract() {
        return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_STAGE_DOWN), elevator);
    }

    public static Command algaeExtend() {
        return Commands.run(() -> elevator.setAlgaeHeightFromGround(ElevatorConstants.ALGAE_STAGE_UP), elevator);
    }

    public static Command algaeRetract() {
        return Commands.run(() -> elevator.setAlgaeHeightFromGround(ElevatorConstants.ALGAE_STAGE_DOWN), elevator);
    }
}