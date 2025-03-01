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
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;
import frc.robot.subsystems.endEffectors.CoralWristEndEffector;

public class GlobalCommands {
  public static Elevator elevator = Elevator.getInstance();
  public static CoralWristEndEffector coralWristEndEffector = CoralWristEndEffector.getInstance();
  public static AlgaeWristEndEffector algaeWristEndEffector = AlgaeWristEndEffector.getInstance();
  
  public static Command humanPlayerIntake() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget());}),
    Commands.parallel(
            CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_INTAKE_IN_VOLTAGE),
            CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_WRIST_DEFAULT_POS),
            ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_HUMAN_PLAYER_INTAKE_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)));
  }

  public static Command algaeGroundIntake() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
    Commands.parallel(
            AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_IN_VOLTAGE),
            AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_GROUND_POS),
            ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_DEFAULT_POS, () -> ElevatorConstants.ALGAE_GROUND_POS)));
  }

  public static Command coralL4() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
    Commands.parallel(
            CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_INTAKE_OUT_VOLTAGE),
            CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L4_POS),
            AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_IN_VOLTAGE),
            AlgaeEndEffectorCommands.algaeWrist(),
            ElevatorCommands.setDynamicL4Pos()));
  }

  public static Command coralL3() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
    Commands.parallel(
            CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_INTAKE_OUT_VOLTAGE),
            CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L3_POS),
            AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_IN_VOLTAGE),
            AlgaeEndEffectorCommands.algaeWrist(),
            ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L3_POS, () -> ElevatorConstants.ALGAE_L2_POS)));

  }

  public static Command coralL2() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget());}),
    Commands.parallel(
            CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_INTAKE_OUT_VOLTAGE),
            CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L2_POS),
            ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L2_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)));

  }

  public static Command coralL1() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget());}),
     Commands.parallel(
            CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_INTAKE_OUT_VOLTAGE),
            CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_L1_POS),
            ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L2_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)));
  }

  public static Command algaeProcessor() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
     Commands.parallel(
            AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_OUT_VOLTAGE),
            AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_PROCESSOR_POS),
            ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L2_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)));
  }

  public static Command algaeBarge() {
    return Commands.deadline(Commands.waitUntil(() -> elevator.isOnTarget()),
     Commands.parallel(
            AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_BARGE_OUT_VOLTAGE),
            AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_BARGE_POS),
            ElevatorCommands.setTargetPosFromZero(() -> ElevatorConstants.CORAL_L2_POS, () -> ElevatorConstants.ALGAE_DEFAULT_POS)));
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
    return Commands.parallel(
      AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_OUT_VOLTAGE));
  }

  public static Command coralRelease() {
    return Commands.parallel(
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
    return CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_WRIST_DEFAULT_POS);
  }

  public static Command defaultAlgaeWristEndEffector() {
    return AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS);
  }
  
  public static Command defaultCoralIntakeEndEffector() {
    return CoralEndEffectorCommands.coralIntake(() -> CoralEndEffectorConstants.CORAL_DEFAULT_VOLTAGE);
  }

  public static Command defaultAlgaeIntakeEndEffector() {
    return AlgaeEndEffectorCommands.algaeIntake(() -> AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE);
  }
}



//   public static Command coralGroundIntakeIn() {
//     return Commands.parallel(
//       GroundIntakeCommands.groundIntakeDown(),
//       GroundIntakeCommands.groundIntakeIn(),
//       ElevatorCommands.coralGroundIntake(),
//       ElevatorCommands.algaeRestingPos()
//     );
//   }

  // public static Command coralGroundIntakeOut() {
  //   return Commands.parallel(
  //     GroundIntakeCommands.groundIntakeOut(),
  //     ElevatorCommands.algaeRestingPos(),
  //     ElevatorCommands.coralGroundIntake()
  //   );
  // }

//   public static Command coralHumanPlayerIntakeIn() {
//     return Commands.parallel(
//       HumanPlayerIntakeCommands.hpIntakeIn(),
//       ElevatorCommands.coralHumanPlayerIntake(),
//       ElevatorCommands.algaeRestingPos()
//     );
//   }

//   public static Command coralHumanPlayerIntakeOut() {
//     return Commands.parallel(
//       HumanPlayerIntakeCommands.hpIntakeOut(),
//       ElevatorCommands.algaeRestingPos(),
//       ElevatorCommands.coralHumanPlayerIntake()
//     );
//   }

//   public static Command scoreCoralL1() {
//     return Commands.parallel(
//       ElevatorCommands.coralL1(),
//       CoralEndEffectorCommands.coralIntakeOut(),
//       ElevatorCommands.algaeRestingPos()
//     );
//   }

//   public static Command scoreCoralL2() {
//     return Commands.parallel(
//       ElevatorCommands.coralL2(),
//       CoralEndEffectorCommands.coralIntakeOut(),
//       ElevatorCommands.algaeRestingPos()
//     );
//   }

//   public static Command scoreCoralL3AndAlgaeL2() {
//     return Commands.parallel(
//       ElevatorCommands.coralL3(),
//       ElevatorCommands.algaeL2(),
//       CoralEndEffectorCommands.coralIntakeOut(),
//       AlgaeEndEffectorCommands.algaeIntakeOut()
//     );
//   }

//   public static Command scoreCoralL4AndAlgaeL3() {
//     return Commands.parallel(
//       ElevatorCommands.coralL4(),
//       ElevatorCommands.algaeL3(),
//       CoralEndEffectorCommands.coralIntakeOut(),
//       AlgaeEndEffectorCommands.algaeIntakeOut()
//     );
//   }

//   public static Command scoreAlgaeBarge() {
//     return Commands.parallel(
//       ElevatorCommands.algaeBarge(),
//       AlgaeEndEffectorCommands.algaeIntakeBargeOut(),
//       ElevatorCommands.coralBarge()
//     );
//   }

//   public static Command scoreAlgaeProcessor() {
//     return Commands.parallel(
//       ElevatorCommands.algaeProcessor(),
//       AlgaeEndEffectorCommands.algaeIntakeOut(),
//       ElevatorCommands.coralRestingPos()
//     );
//   }

//   public static Command extendClimbDeepCage() {
//     return Commands.parallel(
//       ClimbCommands.climbExtend(),
//       ElevatorCommands.algaeRestingPos(),
//       ElevatorCommands.coralRestingPos()
//     );
//   }

//   public static Command retractClimbDeepCage() {
//     return Commands.parallel(
//       ClimbCommands.climbRetract(),
//       ElevatorCommands.algaeRestingPos(),
//       ElevatorCommands.coralRestingPos()
//     );
//   }
// }
