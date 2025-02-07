package frc.robot.subsystems.Constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.interfaces.vision.VisionIOLimelight;
import frc.lib.interfaces.vision.VisionIOPhotonVision;

public class VisionConstants {
  public static final double MAX_AMBIGUITY = 0.25;
  public static final double MAX_Z_ERROR = 0.75; // 0.25
  public static final double MAX_YAW_RATE = 3; // rad/sec
  public static final double MAX_SINGLE_TA = 0.4;

  public static final double LINEAR_STD_BASELINE = 0.3;
  public static final double ANGULAR_STD_BASELINE = 0.3;

  public static final double MT_TRANSLATIONAL_FACTOR = 4;
  public static final double MT_ROTATIONAL_FACTOR = 6;

  public static final double MT2_TRANSLATIONAL_FACTOR = 2.5;
  public static final double MT2_ROTATIONAL_FACTOR = 999999999999999999999999999999999999999999.9;

  public static final double ENABLE_REEF_UPDATES_TA = 0.4;

  public static final VisionIOLimelight LIMELIGHT_LEFT =
      VisionIOLimelight.getInstance(
        "limelight-left",
        new Pose3d(
          new Translation3d(Units.inchesToMeters(-8.625), Units.inchesToMeters(-7.1), Units.inchesToMeters(9.8)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(25), Units.degreesToRadians(-25))
        )
      );
  
  public static final VisionIOLimelight LIMELIGHT_RIGHT =
      VisionIOLimelight.getInstance(
        "limelight-right",
        new Pose3d(
          new Translation3d(Units.inchesToMeters(-8.625), Units.inchesToMeters(7.1), Units.inchesToMeters(9.8)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(25), Units.degreesToRadians(25))
        )
      );

  public static final VisionIOPhotonVision ARDUCAM_ONE = 
    VisionIOPhotonVision.getInstance(
      "Arducam1", 
      new Pose3d(
        new Translation3d(Units.inchesToMeters(7.946071), Units.inchesToMeters(0.929091), Units.inchesToMeters(9.084813 - 0.125)), 
        new Rotation3d(Math.PI, Units.degreesToRadians(32), Math.PI))
    );

  public static final VisionIOPhotonVision ARDUCAM_TWO = 
    VisionIOPhotonVision.getInstance(
      "Arducam2", 
      new Pose3d(
        // new Translation3d(Units.inchesToMeters(7.946071), Units.inchesToMeters(0.929091), Units.inchesToMeters(9.084813 - 0.125)), 
        new Translation3d(),
        new Rotation3d(Math.PI, Units.degreesToRadians(32), Math.PI))
    );
}
