package frc.lib.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  public record TrackedTarget(Translation3d origin, double tx, double ty) {}

  @AutoLog
  public static class VisionIOInputs {

    // Performance information
    public double totalLatencyMs;
    public double fps;

    // Fiducial information
    public Pose3d pinholePose;
    public Pose3d solvePnpPose;

    // Hardware information
    public double cpuTemp;
    public double ram;
    public double temp;
  }

  void setPoseRobotSpace(Pose3d cameraPose);
}
