package frc.robot.subsystems.Constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.generated.TunerConstants;

public class DriveConstants {
  // Pathplanner constants
  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;

  // Pathfinding constants
  public static final double MAX_LINEAR_ACCEL = 1;
  public static final double MAX_ANGULAR_ACCEL = MAX_LINEAR_ACCEL / 2;

  // Pathplanner PID constants
  public static final double PP_TRANSLATION_P = 5.0;
  public static final double PP_TRANSLATION_I = 0.0;
  public static final double PP_TRANSLATION_D = 0.0;
  public static final double PP_HEADING_P = 5.0;
  public static final double PP_HEADING_I = 0.0;
  public static final double PP_HEADING_D = 0.0;

  // Heading Controller
  public static final double HEADING_P = 0.0;
  public static final double HEADING_I = 0.0;
  public static final double HEADING_D = 0.0;

  public static final double HEADING_TOLERANCE_DEGREES = 0.5;

  // Teleop driving constants
  public static final double STICK_DEADBAND = 0.01;
  public static final double STICK_EXPO = 1;

  public static final double TELEOP_MAX_LINEAR_VELOCITY =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double TELEOP_MAX_ANGULAR_VELOCITY = TELEOP_MAX_LINEAR_VELOCITY / 0.5;
}
