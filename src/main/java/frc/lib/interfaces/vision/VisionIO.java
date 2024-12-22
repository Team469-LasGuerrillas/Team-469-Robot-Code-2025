package frc.lib.interfaces.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  public record TrackedTarget(
      double timestamp,
      Translation3d origin,
      double tx,
      double ty,
      double area,
      int fiducialId,
      TargettingType targetType) {}

  public enum TargettingType {
    FIDUCIAL,
    COLOR,
    RETROREFLECTIVE,
    OBJECT_DETECTION
  }

  @AutoLog
  class VisionIOInputs {
    public boolean hasLatestUpdate = false;
    public Pose3d cameraPose;

    // Pipeline
    public boolean hasTargets = false;
    public TargettingType targettingType;

    public TrackedTarget[] targets;

    // Performance information
    public double totalLatencyMs;
    public double fps;

    // Megatag information
    public Pose3d pinholePose;
    public Pose3d solvePnpPose;

    public double[] pinholeStdDevs;
    public double[] solvePnpStdDevs;

    // Hardware information
    public double cpuTemp;
    public double ram;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setPoseRobotSpace(Pose3d cameraPose) {}

  public default void setRobotRotationUpdate(Rotation2d rotation, Rotation2d angularVelocity) {}
}
