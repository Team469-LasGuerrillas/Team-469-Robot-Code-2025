package frc.lib.util;

import frc.robot.subsystems.constants.CoralEndEffectorConstants;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.ElevatorConstants;

public class AutoScore {
    private static DoubleSupplier nextCoralWristPos = () -> CoralEndEffectorConstants.CORAL_L4_POS;
    private static DoubleSupplier nextAlgaeWristPos = () -> AlgaeEndEffectorConstants.ALGAE_WRIST_DEFAULT_POS;
    private static DoubleSupplier nextAlgaeIntakeVol = () -> AlgaeEndEffectorConstants.ALGAE_INTAKE_DEFAULT_VOLTAGE;
    private static DoubleSupplier nextCoralElevatorPos = () -> ElevatorConstants.CORAL_L4_POS;
    private static DoubleSupplier nextAlgaeElevatorPos = () -> ElevatorConstants.ALGAE_L3_POS;

    public static void setNextCoralWristPos(DoubleSupplier newCoralWristPos) { 
        nextCoralElevatorPos = newCoralWristPos; 
    }
    public static DoubleSupplier getNextCoralWristPos() { return nextCoralWristPos; }

    public static void setNextAlgaeWristPos(DoubleSupplier newAlgaeWristPos) { 
        nextAlgaeWristPos = newAlgaeWristPos; 
    }
    public static DoubleSupplier getNextAlgaeWristPos() { return nextAlgaeWristPos; }

    public static void setNextAlgaeIntakeVol(DoubleSupplier newAlgaeIntakeVol) { 
        nextAlgaeIntakeVol = newAlgaeIntakeVol; 
    }
    public static DoubleSupplier getNextAlgaeIntakeVol() { return nextAlgaeIntakeVol; }

    public static void setNextCoralElevatorPos(DoubleSupplier newCoralElevatorPos) { 
        nextCoralElevatorPos = newCoralElevatorPos; 
    }
    public static DoubleSupplier getNextCoralElevatorPos() { return nextCoralElevatorPos; }

    public static void setNextAlgaeElevatorPos(DoubleSupplier newAlgaeElevatorPos) { 
        nextAlgaeElevatorPos = newAlgaeElevatorPos; 
    }
    public static DoubleSupplier getNextAlgaeElevatorPos() { return nextAlgaeElevatorPos; }
}
