package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.AutoScore;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {
    private static Elevator elevator = Elevator.getInstance();

  public static Command setTargetPosFromZero(DoubleSupplier coralPosition, DoubleSupplier algaePosition) {
    return Commands.startRun(() -> elevator.setTargetPosFromZero(coralPosition, algaePosition), () -> elevator.setTargetPosFromZero(coralPosition, algaePosition), elevator);
  }

  public static Command setTargetPosFromZeroAutoScore(double nextCoralPos, double nextAlgaePos) {
    return Commands.run(() -> {
      AutoScore.setNextCoralElevatorPos(nextCoralPos);
      AutoScore.setNextAlgaeElevatorPos(nextAlgaePos);
    }, new Subsystem[]{});
  }

  public static Command setDynamicL4Pos() {
    double algaePosition;
    if (elevator.goToAlgaeL3()) algaePosition = ElevatorConstants.ALGAE_L3_POS;
    else algaePosition = ElevatorConstants.ALGAE_L2_POS;
    
    return Commands.deferredProxy(
      () -> setTargetPosFromZero(
        () -> ElevatorConstants.CORAL_L4_POS, () -> algaePosition)
    );
  }

  public static Command setCoralPosFromZero(DoubleSupplier coralPosition) {
    return Commands.startRun(() -> elevator.setCoralPosFromZero(coralPosition), () -> elevator.setCoralPosFromZero(coralPosition), elevator);
  }

  public static Command setAlgaePosFromZero(DoubleSupplier algaePosition) {
    return Commands.startRun(() -> elevator.setAlgaePosFromZero(algaePosition), () -> elevator.setAlgaePosFromZero(algaePosition), elevator);
  }

  public static Command resetElevatorCommand() {
    return Commands.runOnce(() -> elevator.resetElevatorState(), elevator);
  }

  // JCAO: PLEASE USE WITH CAUTION
  public static Command resetElevatorHighCommand() {
    return Commands.runOnce(() -> elevator.resetElevatorToHighState(), elevator);
  }
}