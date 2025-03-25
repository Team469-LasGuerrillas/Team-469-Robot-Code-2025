package frc.lib.util;

import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;

public class AutoScore {
    public static double nextCoralWristPos = CoralEndEffectorConstants.CORAL_L4_POS;
    public static double nextAlgaeWristPos = AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS;
    public static double nextAlgaeIntakeVol = AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE;
    public static double nextCoralElevatorPos = ElevatorConstants.CORAL_L4_POS;
    public static double nextAlgaeElevatorPos = ElevatorConstants.ALGAE_L3_POS;

    public static void setNextCoralWristPos(double newCoralWristPos) { 
        nextCoralElevatorPos = newCoralWristPos; 
    }

    public static void setNextAlgaeWristPos(double newAlgaeWristPos) { 
        nextAlgaeWristPos = newAlgaeWristPos; 
    }

    public static void setNextAlgaeIntakeVol(double newAlgaeIntakeVol) { 
        nextAlgaeIntakeVol = newAlgaeIntakeVol; 
    }

    public static void setNextCoralElevatorPos(double newCoralElevatorPos) { 
        nextCoralElevatorPos = newCoralElevatorPos; 
    }

    public static void setNextAlgaeElevatorPos(double newAlgaeElevatorPos) { 
        nextAlgaeElevatorPos = newAlgaeElevatorPos; 
    }
}
