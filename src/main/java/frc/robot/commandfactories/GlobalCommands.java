package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.ClimbConstants;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.constants.HumanPlayerIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffectors.AlgaeIntakeEndEffector;
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;
import frc.robot.subsystems.endEffectors.CoralIntakeEndEffector;
import frc.robot.subsystems.endEffectors.CoralWristEndEffector;

public class GlobalCommands {
  public static Elevator elevator = Elevator.getInstance();
  public static CoralWristEndEffector coralWristEndEffector = CoralWristEndEffector.getInstance();
  public static AlgaeWristEndEffector algaeWristEndEffector = AlgaeWristEndEffector.getInstance();
  
  public static Command humanPlayerIntake() {
    return Commands.parallel(
      CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_INTAKE_IN_VOLTAGE),
      CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_HP_INTAKE_POS)
      // ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_HUMAN_PLAYER_INTAKE_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)
    );
  }

  public static Command hackyHumanPlayerIntake() {
    return Commands.parallel(
      CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_INTAKE_IN_VOLTAGE),
      CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_HP_INTAKE_POS),
      ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_HUMAN_PLAYER_INTAKE_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)
    );
  }

  public static Command algaeGroundIntake() {
    return Commands.deadline(
      Commands.waitUntil(() -> {return (false && elevator.isCoralOnTarget() && algaeWristEndEffector.isOnTarget());}),
      AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_IN_VOLTAGE),
      AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_GROUND_POS)
    );
  }

  public static Command coralL4() {
    return Commands.deadline(Commands.waitUntil(() -> {return false && (elevator.isCoralOnTarget() && coralWristEndEffector.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
            CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L4_POS),
            AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_IN_VOLTAGE),
            AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_L2_L3),
            ElevatorCommands.setDynamicL4Pos());
  }

  public static Command coralL4NoAlgae() {
    return Commands.deadline(Commands.waitUntil(() -> {return false && (elevator.isCoralOnTarget() && coralWristEndEffector.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
            CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L4_POS),
            ElevatorCommands.setDynamicL4Pos());
  }

  public static Command coralL3() {
    return Commands.deadline(Commands.waitUntil(() -> {return false && (elevator.isCoralOnTarget() && coralWristEndEffector.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
      CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L3_POS - 0.05),
      AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_IN_VOLTAGE),
      AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_L2_L3),
      ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L3_POS, () -> ElevatorConstants.ALGAE_L2_POS)
    );

  }

  public static Command coralL3NoAlgae() {
    return Commands.deadline(Commands.waitUntil(() -> {return false && (elevator.isCoralOnTarget() && coralWristEndEffector.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
      CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L3_POS),
      ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L3_POS, () -> ElevatorConstants.ALGAE_L2_POS)
    );

  }

  public static Command coralL2() {
    return Commands.deadline(Commands.waitUntil(() -> {return false && (elevator.isCoralOnTarget() && coralWristEndEffector.isOnTarget());}),
      CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L2_POS),
      ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L2_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)
    );

  }

  public static Command coralL1() {
    // TODO: Remove falsse && in end condition check (JCAO: Forcing command to never end)
    return Commands.deadline(Commands.waitUntil(() -> {return false && (elevator.isCoralOnTarget() && coralWristEndEffector.isOnTarget());}),
      CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L1_POS),
      ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L1_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)
    );
  }

  public static Command algaeProcessor() {
    return Commands.deadline(Commands.waitUntil(() -> {return false && (elevator.isCoralOnTarget() && algaeWristEndEffector.isOnTarget());}),
      // CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_PROCESSOR_POS),
      AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_PROCESSOR_POS),
      ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_PROCESSOR_POS, () -> ElevatorConstants.ALGAE_PROCESSOR_POS)
    );
  }

  public static Command algaeBarge() {
    return Commands.deadline(Commands.waitUntil(() -> false && elevator.isCoralOnTarget()),
      CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_BARGE_POS),
      AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_BARGE_POS),
      ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_BARGE_POS, () -> ElevatorConstants.ALGAE_BARGE_POS)
    );
  }

  public static Command slowRetract() {
    return ClimbCommands.climb(() -> ClimbConstants.SLOW_RETRACT);
  } 

  public static Command fastRetract() {
    return ClimbCommands.climb(() -> ClimbConstants.FAST_RETRACT);
  }

  public static Command deploy() {
    return ClimbCommands.climb(() -> ClimbConstants.DEPLOY);
  }

  public static Command algaeRelease() {
    return Commands.deadline(
      Commands.waitSeconds(0.25),
      AlgaeEndEffectorCommands.algaeIntake(() ->  AlgaeEndEffectorConstants.ALGAE_INTAKE_OUT_VOLTAGE));
  }

  public static Command coralRelease() {
    return Commands.deadline(
      Commands.waitSeconds(0.5),
      CoralEndEffectorCommands.coralIntake(() ->  CoralEndEffectorConstants.CORAL_INTAKE_OUT_VOLTAGE));
  }

  /* DEFAULT POSITIONS */
  public static Command defaultAlgaeElevatorPosition() {
    return ElevatorCommands.setAlgaePosFromZero(() -> ElevatorConstants.ALGAE_DEFAULT_POS);
  }

  public static Command defaultCoralElevatorPosition() {
    return ElevatorCommands.setCoralPosFromZero(() -> ElevatorConstants.CORAL_DEFAULT_POS);
  }

  public static Command defaultCoralWristEndEffector() {
    return CoralEndEffectorCommands.coralWristDefault();
  }

  public static Command defaultAlgaeWristEndEffector() {
    return AlgaeEndEffectorCommands.algaeWristDefault();
  }
  
  public static Command defaultCoralIntakeEndEffector() {
    return CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_DEFAULT_VOLTAGE);
  }

  public static Command defaultAlgaeIntakeEndEffector() {
    return AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE);
  }

  public static Command defaultClimb() {
    return ClimbCommands.climb(() -> 0);
  }

  public static Command defaultElevator() {
    return ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_DEFAULT_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS);
  }
}