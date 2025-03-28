package frc.lib.util;

import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;

public class AutoScore {
    private static double nextCoralWristPos = CoralEndEffectorConstants.CORAL_L4_POS;
    private static double nextAlgaeWristPos = AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS;
    private static double nextAlgaeIntakeVol = AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE;
    private static double nextCoralElevatorPos = ElevatorConstants.CORAL_L4_POS;
    private static double nextAlgaeElevatorPos = ElevatorConstants.ALGAE_L3_POS;

    public static void setNextCoralWristPos(double newCoralWristPos) { 
        nextCoralElevatorPos = newCoralWristPos; 
    }
    public static double getNextCoralWristPos() { return nextCoralWristPos; }

    public static void setNextAlgaeWristPos(double newAlgaeWristPos) { 
        nextAlgaeWristPos = newAlgaeWristPos; 
    }
    public static double getNextAlgaeWristPos() { return nextAlgaeWristPos; }

    public static void setNextAlgaeIntakeVol(double newAlgaeIntakeVol) { 
        nextAlgaeIntakeVol = newAlgaeIntakeVol; 
    }
    public static double getNextAlgaeIntakeVol() { return nextAlgaeIntakeVol; }

    public static void setNextCoralElevatorPos(double newCoralElevatorPos) { 
        nextCoralElevatorPos = newCoralElevatorPos; 
    }
    public static double getNextCoralElevatorPos() { return nextCoralElevatorPos; }

    public static void setNextAlgaeElevatorPos(double newAlgaeElevatorPos) { 
        nextAlgaeElevatorPos = newAlgaeElevatorPos; 
    }
    public static double getNextAlgaeElevatorPos() { return nextAlgaeElevatorPos; }
}
