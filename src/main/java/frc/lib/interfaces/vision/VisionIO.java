package frc.lib.interfaces.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  public record TrackedTarget(
    double timestamp, 
    Pose3d pose,
    double ambiguity, 
    int tagCount, 
    double averageTagDistance,
    double tx,
    double ty,
    double area,
    int fiducialId,
    TargetType type
  ) {}

  public enum TargetType {
    MEGATAG_1, // Limelight
    MEGATAG_2, // Limelight
    MULTI_TAG // PhotonVision
  }

  @AutoLog
  class VisionIOInputs {
    // Target Information
    public TrackedTarget[] targets = new TrackedTarget[0];
    public boolean hasTargets = false;

    // Performance Metrics
    public double totalLatencyMs;
    public double fps;

    // Hardware Information
    public double cpuTemp;
    public double ram;
    public double temp;
    public boolean connected = false;

    // Limelight-specific information
    public double[] pinholeStdDevs;
    public double[] solvePnpStdDevs;
  }

  void updateInputs(VisionIOInputs inputs);
}

// package frc.lib.interfaces.vision;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import frc.lib.util.TargettingType;
// import org.littletonrobotics.junction.AutoLog;

// public interface VisionIO {

//   public record TrackedTarget(
//       TargettingType targetType,
//       Translation3d origin,
//       double tx,
//       double ty,
//       double area,
//       int fiducialId) {}

//   @AutoLog
//   class VisionIOInputs {
//     public boolean connected = false;

//     // Pipeline
//     public boolean hasTargets = false;
//     public boolean hasLatestUpdate = false;
//     public TargettingType targettingType;

//     public TrackedTarget[] targets;

//     // Performance information
//     public double totalLatencyMs;
//     public double fps;

//     // Megatag information
//     public Pose3d pinholePose;
//     public Pose3d solvePnpPose;

//     public double[] pinholeStdDevs;
//     public double[] solvePnpStdDevs;

//     // Hardware information
//     public double cpuTemp;
//     public double ram;
//     public double temp;
//   }

//   void updateInputs(VisionIOInputs inputs);

//   void setPoseRobotSpace(Pose3d cameraPose);

//   void setRobotRotationUpdate(Rotation2d rotation, Rotation2d angularVelocity);
// }