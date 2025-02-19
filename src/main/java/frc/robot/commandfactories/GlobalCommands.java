package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class GlobalCommands {
  public static Command coralGroundIntakeIn() {
    return Commands.parallel(
      GroundIntakeCommands.groundIntakeDown(),
      GroundIntakeCommands.groundIntakeIn(),
      ElevatorCommands.coralGroundIntake(),
      ElevatorCommands.algaeRestingPos()
    );
  }

  public static Command coralGroundIntakeOut() {
    return Commands.parallel(
      GroundIntakeCommands.groundIntakeOut(),
      ElevatorCommands.algaeRestingPos(),
      ElevatorCommands.coralGroundIntake()
    );
  }

  public static Command coralHumanPlayerIntakeIn() {
    return Commands.parallel(
      HumanPlayerIntakeCommands.hpIntakeIn(),
      ElevatorCommands.coralHumanPlayerIntake(),
      ElevatorCommands.algaeRestingPos()
    );
  }

  public static Command coralHumanPlayerIntakeOut() {
    return Commands.parallel(
      HumanPlayerIntakeCommands.hpIntakeOut(),
      ElevatorCommands.algaeRestingPos(),
      ElevatorCommands.coralHumanPlayerIntake()
    );
  }

  public static Command scoreCoralL1() {
    return Commands.parallel(
      ElevatorCommands.coralL1(),
      CoralEndEffectorCommands.coralIntakeOut(),
      ElevatorCommands.algaeRestingPos()
    );
  }

  public static Command scoreCoralL2() {
    return Commands.parallel(
      ElevatorCommands.coralL2(),
      CoralEndEffectorCommands.coralIntakeOut(),
      ElevatorCommands.algaeRestingPos()
    );
  }

  public static Command scoreCoralL3AndAlgaeL2() {
    return Commands.parallel(
      ElevatorCommands.coralL3(),
      ElevatorCommands.algaeL2(),
      CoralEndEffectorCommands.coralIntakeOut(),
      AlgaeEndEffectorCommands.algaeIntakeOut()
    );
  }

  public static Command scoreCoralL4AndAlgaeL3() {
    return Commands.parallel(
      ElevatorCommands.coralL4(),
      ElevatorCommands.algaeL3(),
      CoralEndEffectorCommands.coralIntakeOut(),
      AlgaeEndEffectorCommands.algaeIntakeOut()
    );
  }

  public static Command scoreAlgaeBarge() {
    return Commands.parallel(
      ElevatorCommands.algaeBarge(),
      AlgaeEndEffectorCommands.algaeIntakeBargeOut(),
      ElevatorCommands.coralBarge()
    );
  }

  public static Command scoreAlgaeProcessor() {
    return Commands.parallel(
      ElevatorCommands.algaeProcessor(),
      AlgaeEndEffectorCommands.algaeIntakeOut(),
      ElevatorCommands.coralRestingPos()
    );
  }

  public static Command extendClimbDeepCage() {
    return Commands.parallel(
      ClimbCommands.climbExtend(),
      ElevatorCommands.algaeRestingPos(),
      ElevatorCommands.coralRestingPos()
    );
  }

  public static Command retractClimbDeepCage() {
    return Commands.parallel(
      ClimbCommands.climbRetract(),
      ElevatorCommands.algaeRestingPos(),
      ElevatorCommands.coralRestingPos()
    );
  }
}
