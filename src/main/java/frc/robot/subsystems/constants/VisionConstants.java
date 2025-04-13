package frc.robot.subsystems.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.interfaces.vision.VisionIOLimelight;
import frc.lib.interfaces.vision.VisionIOPhotonVision;

public class VisionConstants {
  public static final double MAX_AMBIGUITY = 0.55;
  public static final double MAX_Z_ERROR = 0.5; // 0.25
  public static final double MAX_ROTATION_ERROR_DEGREES = 2;
  public static final double MAX_YAW_RATE = 3; // rad/sec
  public static final double MAX_MEGATAG_SINGLE_TA = 0.4;
  public static final double MAX_MULITAG_SINGLE_TA = 0.2;

  public static final double MULTITAG_TRANSLATIONAL_FACTOR = 0.01;
  public static final double MULTITAG_ROTATIONAL_FACTOR = 1;

  public static final double MULTITAG2_TRANSLATIONAL_FACTOR = 0.00469;
  public static final double MULITAG2_ROTATIONAL_FACTOR = 1.7976E26;

  public static final double MEGATAG_TRANSLATIONAL_FACTOR = 1;
  public static final double MEGATAG_ROTATIONAL_STD = 0.0469;

  public static final double MEGATAG2_TRANSLATIONAL_FACTOR = 0.8;
  public static final double MEGATAG2_ROTATIONAL_FACTOR = 1.7976E308;

  public static final double ENABLE_REEF_UPDATES_TA = 0.4;
  public static final double LAST_GOOD_UPDATE_TIME_THRESHOLD = 0.469;
  public static final double LOOKING_AT_REEF_THRESHOLD_DEGREES = 78;
  public static final double REEF_RADIUS = 2.5;
  public static final double SMALL_REEF_RADIUS = 2;

  public static final int[] REEF_TAG_IDS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  public static final int[] ALL_TAG_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};

  public static final VisionIOLimelight LIMELIGHT_LEFT =
      VisionIOLimelight.getInstance(
        "limelight-left",
        new Pose3d(
          new Translation3d(Units.inchesToMeters(-11.984), Units.inchesToMeters(4.982), Units.inchesToMeters(6.046 - 0.125)), // minus an eighth for carpet squish
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(25), Units.degreesToRadians(215))
        )
      );
  
  public static final VisionIOLimelight LIMELIGHT_RIGHT =
      VisionIOLimelight.getInstance(
        "limelight-right",
        new Pose3d(
          new Translation3d(Units.inchesToMeters(-11.984), Units.inchesToMeters(-4.982), Units.inchesToMeters(6.046 - 0.125)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(25), Units.degreesToRadians(-215))
        )
      );

  public static final VisionIOLimelight LIMELIGHT_CENTER =
  VisionIOLimelight.getInstance(
    "limelight-center",
    new Pose3d(
      new Translation3d(Units.inchesToMeters(1.898941), Units.inchesToMeters(0), Units.inchesToMeters(38.788085 - 0.125)),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(0))
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
