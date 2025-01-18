package frc.robot.subsystems.Constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.interfaces.vision.VisionIO;
import frc.lib.interfaces.vision.VisionIOLimelight;

public class VisionConstants {
  public static final double MAX_AMBIGUITY = 0.25;
  public static final double MAX_Z_ERROR = 0.25;
  public static final double MAX_YAW_RATE = 3; // rad/sec

  public static final double LINEAR_STD_BASELINE = 0.3;
  public static final double ANGULAR_STD_BASELINE = 0.3;

  public static final double LIMELIGHT_TRANSLATIONAL_STD_FACTOR = 2;
  public static final double LIMELIGHT_ROTATIONAL_STD_FACTOR = 2;

  public static final VisionIOLimelight LIMELIGHT_LEFT =
      VisionIOLimelight.getInstance(
        "limelight-left",
        new Pose3d(
          new Translation3d(Units.inchesToMeters(-8.75), Units.inchesToMeters(-7), Units.inchesToMeters(9.8)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(25), Units.degreesToRadians(-25))
        )
      );
  
  public static final VisionIOLimelight LIMELIGHT_RIGHT =
      VisionIOLimelight.getInstance(
        "limelight-right",
        new Pose3d(
          new Translation3d(Units.inchesToMeters(-8.75), Units.inchesToMeters(7), Units.inchesToMeters(9.8)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(25), Units.degreesToRadians(25))
        )
      );

}
