// package frc.robot.commandfactories;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.constants.ElevatorConstants;
// import frc.robot.subsystems.elevator.Elevator;

// public class ElevatorCommands {
//     private static Elevator elevator = Elevator.getInstance();

//     public static Command coralRestingPos() {
//         // return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_RESTING_POS), elevator);
//     }

//     public static Command coralGroundIntake() {
//         // return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_GROUND_INTAKE), elevator);
//     }
    
//     public static Command coralHumanPlayerIntake() {
//         // return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_HUMAN_PLAYER_INTAKE), elevator);
//     }

//     public static Command coralL1() {
//         // return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_L1), elevator);
//     }

//     public static Command coralL2() {
//         return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_L2), elevator);
//     }

//     public static Command coralL3() {
//         return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_L3), elevator);
//     }

//     public static Command coralL4() {
//         return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_L4), elevator);
//     }

//     public static Command coralBarge() {
//         return Commands.run(() -> elevator.setCoralHeightFromGround(ElevatorConstants.CORAL_BARGE), elevator);
//     }

//     public static Command algaeRestingPos() {
//         return Commands.run(() -> elevator.setAlgaeHeightFromGround(ElevatorConstants.ALGAE_RESTING_POS), elevator);
//     }

//     public static Command algaeL2() {
//         return Commands.run(() -> elevator.setAlgaeHeightFromGround(ElevatorConstants.ALGAE_L2), elevator);
//     }

//     public static Command algaeL3() {
//         return Commands.run(() -> elevator.setAlgaeHeightFromGround(ElevatorConstants.ALGAE_L3), elevator);
//     }

//     public static Command algaeProcessor() {
//         return Commands.run(() -> elevator.setAlgaeHeightFromGround(ElevatorConstants.ALGAE_PROCESSOR), elevator);
//     }

//     public static Command algaeBarge() {
//         return Commands.run(() -> elevator.setAlgaeHeightFromGround(ElevatorConstants.ALGAE_BARGE), elevator);
//     }
// }