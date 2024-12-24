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

  public record PoseObservation(
      double timestamp,
      Pose3d pinholePose,
      Pose3d solvePnpPose,
      double[] pinholeStdDevs,
      double[] solvePnpStdDevs) {}

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
    public PoseObservation[] poseObservations;

    // Performance information
    public double totalLatencyMs;
    public double fps;

    // Hardware information
    public double cpuTemp;
    public double ram;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setPoseRobotSpace(Pose3d cameraPose) {}

  public default void setRobotRotationUpdate(Rotation2d rotation, Rotation2d angularVelocity) {}

  public default void setPipelineIndex(int index) {}
}
