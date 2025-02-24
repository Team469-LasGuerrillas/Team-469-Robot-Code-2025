package frc.robot.commandfactories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;
import frc.robot.subsystems.endEffectors.CoralWristEndEffector;

public class GlobalCommands {
  public static Elevator elevator = Elevator.getInstance();
  public static CoralWristEndEffector coralWristEndEffector = CoralWristEndEffector.getInstance();
  public static AlgaeWristEndEffector algaeWristEndEffector = AlgaeWristEndEffector.getInstance();

  /* EXAMPLE */
  public static Command hehe() {
    // () -> variable
    // () -> {return variable;}

    // () -> {return (Elevator.getInstance().isOnTarget() && CoralWristEndEffector.getInstance().isOnTarget());}
    return Commands.deadline(
      Commands.waitUntil(() -> {return (Elevator.getInstance().isOnTarget() && CoralWristEndEffector.getInstance().isOnTarget());}), 
      Commands.none()
    );
  }


  
  public static Command humanPlayerIntake() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget());}),
     Commands.none());
  }

  public static Command scoreProcessor() {
    return Commands.deadline(Commands.waitUntil(() -> elevator.isOnTarget()),
     Commands.none());
  }

  public static Command coralL4AlgaeL3() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
     Commands.none());
  }

  public static Command coralL3AlgaeL2() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget() && algaeWristEndEffector.isOnTarget());}),
     Commands.none());
  }

  public static Command coralL2() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget());}),
     Commands.none());
  }

  public static Command coralL1() {
    return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget());}),
     Commands.none());
  }

  // public static Command groundIntake() {
  //   return Commands.deadline(Commands.waitUntil(() -> {return (elevator.isOnTarget() && coralWristEndEffector.isOnTarget());}),
  //    Commands.none());
  // }

  /* DEFAULT POSITIONS */
  public static Command defaultAlgaeElevatorPosition() {
    return ElevatorCommands.setAlgaePosFromZero(() -> ElevatorConstants.ALGAE_RESTING_POS);
  }

  public static Command defaultCoralElevatorPosition() {
    return ElevatorCommands.setCoralPosFromZero(() -> ElevatorConstants.CORAL_RESTING_POS);
  }

  public static Command defaultCoralWristEndEffector() {
    return CoralEndEffectorCommands.coralWrist(() -> CoralEndEffectorConstants.CORAL_WRIST_DEFAULT_POSITION);
  }

  public static Command defaultAlgaeWristEndEffector() {
    return AlgaeEndEffectorCommands.algaeWrist(() -> AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT);
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

//   public static Command coralGroundIntakeOut() {
//     return Commands.parallel(
//       GroundIntakeCommands.groundIntakeOut(),
//       ElevatorCommands.algaeRestingPos(),
//       ElevatorCommands.coralGroundIntake()
//     );
//   }

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
