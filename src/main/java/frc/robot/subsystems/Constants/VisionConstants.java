package frc.robot.subsystems.Constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.interfaces.vision.VisionIO;
import frc.lib.interfaces.vision.VisionIOLimelight;

public class VisionConstants {
  public static final double MAX_AMBIGUITY = 0.0;
  public static final double MAX_Z_ERROR = 10;

  public static final double LINEAR_STD_BASELINE = 0.3;
  public static final double ANGULAR_STD_BASELINE = 0.3;

  public static final VisionIOLimelight LIMELIGHT_LEFT = VisionIOLimelight.getInstance("limelight-left", new Pose3d(new Translation3d(-8.75, -7, 9.8), new Rotation3d(0, 25, -25)));
  
  public static final VisionIOLimelight LIMELIGHT_RIGHT = VisionIOLimelight.getInstance("limelight-right", new Pose3d(new Translation3d(-8.75, 7, 9.8), new Rotation3d(0, 25, 25)));

}
