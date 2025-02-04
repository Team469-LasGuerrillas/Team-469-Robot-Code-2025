package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

public class State extends Command{

    public enum RobotState {
        CORAL_GROUND_INTAKE_IN,
        CORAL_GROUND_INTAKE_OUT,
        ALGAE_INTAKE_IN,
        CLIMB_EXTEND,
        CLIMB_RETRACT,
        CORAL_L4_AND_ALGAE_L3,
        CORAL_L4_AND_ALGAE_L2,
        CORAL_L3_AND_ALGAE_L2,
        CORAL_L2,
        CORAL_L1,
        ALGAE_PROCESSOR,
        ALGAE_NET,
        CORAL_END_EFFECTOR_INTAKE,
        CORAL_END_EFFECTOR_SCORE,
        ALGAE_END_EFFECTOR_SCORE,
        HUMAN_PLAYER_INTAKE_IN,
        HUMAN_PLAYER_INTAKE_OUT,
        DEFAULT_CORAL,
    }
}
