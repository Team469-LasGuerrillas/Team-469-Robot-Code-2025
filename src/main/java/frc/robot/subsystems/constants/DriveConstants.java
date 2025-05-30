package frc.robot.subsystems.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  public static final double DRIVE_BASE_RADIUS =
  Math.max(
      Math.max(
          Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
      Math.max(
          Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
    
  // Pathplanner constants
  public static final double ROBOT_MASS_KG = 65.77;
  public static final double ROBOT_MOI = 2; // 4.5
  public static final double WHEEL_COF = 1.8;

  // Pathfinding constants
  public static final double MAX_LINEAR_ACCEL_FINE = 0.4; // 0.09
  public static final double MAX_LINEAR_ACCEL_GROSS = 0.23; // 0.16
  public static final double MAX_ANGULAR_ACCEL = 0.125;

  // Pathplanner PID constants
  public static final double PP_TRANSLATION_P = 9;
  public static final double PP_TRANSLATION_I = 2;
  public static final double PP_TRANSLATION_D = 0.0;
  public static final double PP_HEADING_P = 5;
  public static final double PP_HEADING_I = 1;
  public static final double PP_HEADING_D = 0.0;

  // Heading Controller
  public static final double HEADING_P = 4;
  public static final double HEADING_I = 0.0;
  public static final double HEADING_D = 0;

  public static final double HEADING_TOLERANCE_DEGREES = 0.4;
  public static final double L1_HEADING_TOLERANCE_DEGREES = 4;
  public static final double HP_HEADING_TOLERANCE_DEGREES = 3;
  public static final double HEADING_TOLERANCE_TO_SCORE_DEGREES = 1.5;

  public static final double HEADING_TOLERANCE_TO_RAISE_ELEVATOR = 90;
  public static final double HEADING_TOLERANCE_TO_START_HP_INTAKE = 20;

  // Linear Controller
  public static final double MIN_SPEED_FOR_OUTPUT = 0.000001;

  public static final double LINEAR_P_FINE = 3;
  public static final double LINEAR_I_FINE = 0.7;
  public static final double LINEAR_D_FINE = 0.4;

  public static final double LINEAR_P_GROSS = 3;
  public static final double LINEAR_I_GROSS = 0;
  public static final double LINEAR_D_GROSS = 1.5;

  public static final double LINEAR_TOLERANCE_METERS = 0.005;
  public static final double L1_LINEAR_TOLERANCE_METERS = 0.2;
  public static final double HP_LINEAR_TOLERANCE_METERS = 0.075;
  public static final double LINEAR_TOLERACE_TO_SCORE_METERS = 0.025;
  public static final double I_ZONE_METERS = 0.1;
  public static final double ELEVATOR_TOLERANCE_BEFORE_DRIVING_TO_HP = 65;

  public static final double LINEAR_TOLERANCE_TO_RAISE_ELEVATOR = 2.125; // 1.625
  public static final double LINEAR_TOLERANCE_TO_START_HP_INTAKE = 1;

  public static final double FIELD_VELOCITY_CORRECTION_FACTOR_MAGIC_NUMBER = 1.0;

  public static final double ERROR_FOR_DYNAMIC_ELEVATOR_METERS = Units.inchesToMeters(4.25);
  // Teleop driving constants
  public static final double STICK_DEADBAND = 0.01;
  public static final double TRIGGER_DEADBAND = 0.25;
  public static final double STICK_EXPO = 1;

  public static final double TELEOP_MAX_LINEAR_VELOCITY =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double TELEOP_MAX_ANGULAR_VELOCITY = TELEOP_MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS;

  public static final double AUTO_VELOCITY = TELEOP_MAX_LINEAR_VELOCITY;

  public static final Matrix<N3, N1> FUSED_STD_DEVS = VecBuilder.fill(0.025, 0.025, 0.05);
  public static final Matrix<N3, N1> VR_STD_DEVS = VecBuilder.fill(0.01, 0.01, 0.01);
  public static final Matrix<N3, N1> WHEEL_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.01);
}
