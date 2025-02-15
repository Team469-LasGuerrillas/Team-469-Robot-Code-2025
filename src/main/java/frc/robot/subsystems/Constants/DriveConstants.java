package frc.robot.subsystems.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  // Pathplanner constants
  public static final double ROBOT_MASS_KG = 19.5;
  public static final double ROBOT_MOI = 0.41;
  public static final double WHEEL_COF = 1.5;

  // Pathfinding constants
  public static final double MAX_LINEAR_ACCEL = 5;
  public static final double MAX_ANGULAR_ACCEL = MAX_LINEAR_ACCEL / 2;

  // Pathplanner PID constants
  public static final double PP_TRANSLATION_P = 12;
  public static final double PP_TRANSLATION_I = 1;
  public static final double PP_TRANSLATION_D = 0.0;
  public static final double PP_HEADING_P = 7.5;
  public static final double PP_HEADING_I = 1;
  public static final double PP_HEADING_D = 0.0;

  // Heading Controller
  public static final double HEADING_P = 5.0;
  public static final double HEADING_I = 0.0;
  public static final double HEADING_D = 0.0;

  public static final double HEADING_TOLERANCE_DEGREES = 1;

  // Linear Controller
  public static final double LINEAR_P = 5.0;
  public static final double LINEAR_I = 0.0;
  public static final double LINEAR_D = 0.0;

  public static final double LINEAR_TOLERANCE_METERS = 0.000005;

  // Teleop driving constants
  public static final double STICK_DEADBAND = 0.01;
  public static final double STICK_EXPO = 1;

  public static final double TELEOP_MAX_LINEAR_VELOCITY =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double TELEOP_MAX_ANGULAR_VELOCITY = TELEOP_MAX_LINEAR_VELOCITY / 0.5;

  public static final Matrix<N3, N1> FUSED_STD_DEVS = VecBuilder.fill(0.025, 0.025, 0.05);
  public static final Matrix<N3, N1> VR_STD_DEVS = VecBuilder.fill(0.01, 0.01, 0.01);
  public static final Matrix<N3, N1> WHEEL_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.01);
}
