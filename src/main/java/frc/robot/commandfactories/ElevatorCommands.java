package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {
    private static Elevator elevator = Elevator.getInstance();

  public static Command setTargetPosFromZero(DoubleSupplier coralPosition, DoubleSupplier algaePosition) {
    return Commands.run(() -> elevator.setTargetPosFromZero(coralPosition, algaePosition));
  }

  public static Command setCoralPosFromZero(DoubleSupplier coralPosition) {
    return Commands.run(() -> elevator.setCoralPosFromZero(coralPosition));
  }

  public static Command setAlgaePosFromZero(DoubleSupplier algaePosition) {
    return Commands.run(() -> elevator.setAlgaePosFromZero(algaePosition));
  }
}