package frc.lib.util;

import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;

public class AutoScore {
    private static Double nextCoralWristPos = CoralEndEffectorConstants.CORAL_L4_POS;
    private static Double nextAlgaeWristPos = AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS;
    private static Double nextAlgaeIntakeVol = AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE;
    private static Double nextCoralElevatorPos = ElevatorConstants.CORAL_L4_POS;
    private static Double nextAlgaeElevatorPos = ElevatorConstants.ALGAE_L3_POS;

    public static void setNextCoralWristPos(Double newCoralWristPos) { 
        nextCoralWristPos = newCoralWristPos; 
    }
    public static Double getNextCoralWristPos() { return nextCoralWristPos; }

    public static void setNextAlgaeWristPos(Double newAlgaeWristPos) { 
        nextAlgaeWristPos = newAlgaeWristPos; 
    }
    public static Double getNextAlgaeWristPos() { return nextAlgaeWristPos; }

    public static void setNextAlgaeIntakeVol(Double newAlgaeIntakeVol) { 
        nextAlgaeIntakeVol = newAlgaeIntakeVol; 
    }
    public static Double getNextAlgaeIntakeVol() { return nextAlgaeIntakeVol; }

    public static void setNextCoralElevatorPos(Double newCoralElevatorPos) { 
        nextCoralElevatorPos = newCoralElevatorPos; 
    }
    public static Double getNextCoralElevatorPos() { return nextCoralElevatorPos; }

    public static void setNextAlgaeElevatorPos(Double newAlgaeElevatorPos) { 
        nextAlgaeElevatorPos = newAlgaeElevatorPos; 
    }
    public static Double getNextAlgaeElevatorPos() { return nextAlgaeElevatorPos; }

    public static void resetAutoScoreToL4() {
        setNextCoralWristPos(CoralEndEffectorConstants.CORAL_L4_POS);
        setNextAlgaeWristPos(AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS);
        setNextAlgaeIntakeVol(AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE);
        setNextCoralElevatorPos(ElevatorConstants.CORAL_L4_POS);
        setNextAlgaeElevatorPos(ElevatorConstants.ALGAE_L3_POS);
    }
}
