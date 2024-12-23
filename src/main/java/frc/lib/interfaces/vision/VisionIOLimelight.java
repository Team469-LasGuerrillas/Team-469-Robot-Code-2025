package frc.lib.interfaces.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.frc2025.util.LimelightHelpers;
import frc.frc2025.util.LimelightHelpers.RawDetection;
import frc.frc2025.util.LimelightHelpers.RawFiducial;
import frc.lib.util.Clock;
import java.util.HashMap;

public class VisionIOLimelight implements VisionIO {
  static HashMap<String, VisionIOLimelight> instances = new HashMap<String, VisionIOLimelight>();

  private final String limelightName;
  private double lastHeartbeat = 0;

  RawFiducial[] fiducials;
  RawDetection[] detections;

  private double totalLatencyMs;
  private Pose3d cameraPose;

  /**
   * Updated for Limelight OS 2024.10.1, 20240915
   *
   * @param limelightName Name of the limelight, configurable via web GUI. Should look something
   *     like limelight-customname.
   */
  public VisionIOLimelight(String limelightName) {
    this.limelightName = limelightName;

    fiducials = LimelightHelpers.getRawFiducials(limelightName);
    detections = LimelightHelpers.getRawDetections(limelightName);
  }

  public static VisionIOLimelight getInstance(String limelightName) {
    if (instances.get(limelightName) == null) {
      instances.put(limelightName, new VisionIOLimelight(limelightName));
    }
    return instances.get(limelightName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double heartbeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");

    inputs.hasLatestUpdate = lastHeartbeat < heartbeat;
    lastHeartbeat = heartbeat;
    inputs.cameraPose = LimelightHelpers.getCameraPose3d_RobotSpace(limelightName);
    cameraPose = inputs.cameraPose;

    if (LimelightHelpers.getLimelightNTDouble(limelightName, "tv") == 1) {
      inputs.hasTargets = true;
    } else {
      inputs.hasTargets = false;
    }

    inputs.targettingType = getPipeType();
    inputs.totalLatencyMs =
        LimelightHelpers.getLatency_Capture(limelightName)
            + LimelightHelpers.getLatency_Pipeline(limelightName);
    totalLatencyMs = inputs.totalLatencyMs;

    inputs.targets = parseTargets();

    double[] hw = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "hw");
    inputs.fps = hw[0];
    inputs.cpuTemp = hw[1];
    inputs.ram = hw[2];

    inputs.pinholePose = LimelightHelpers.getBotPose3d_wpiBlue_MegaTag2(limelightName);
    inputs.solvePnpPose = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);

    double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "stddevs");
    inputs.pinholeStdDevs = new double[] {stddevs[6], stddevs[7], stddevs[11]};
    inputs.solvePnpStdDevs = new double[] {stddevs[0], stddevs[1], stddevs[5]};
  }

  private TargettingType getPipeType() {
    String pipeString = LimelightHelpers.getCurrentPipelineType(limelightName);

    TargettingType type;

    switch (pipeString) {
      case "pipe_fiducial":
        type = TargettingType.FIDUCIAL;
        break;

      case "pipe_color":
        type = TargettingType.COLOR;

      case "pipe_neuralnetwork":
        type = TargettingType.OBJECT_DETECTION;

      default:
        type = TargettingType.RETROREFLECTIVE;
        break;
    }

    return type;
  }

  private TrackedTarget[] parseTargets() {
    int numberOfTargets = fiducials.length + detections.length;
    TrackedTarget[] targets = new TrackedTarget[numberOfTargets];

    for (int i = 0; i < fiducials.length; i++) {
      targets[i] =
          new TrackedTarget(
              Clock.time() - Units.millisecondsToSeconds(totalLatencyMs),
              cameraPose.getTranslation(),
              cameraPose.getRotation().getX() - Units.degreesToRadians(fiducials[i].txnc),
              cameraPose.getRotation().getY() + Units.degreesToRadians(fiducials[i].tync),
              fiducials[i].ta,
              fiducials[i].id,
              TargettingType.FIDUCIAL);
    }

    for (int i = 0; i < detections.length; i++) {
      targets[i + fiducials.length] =
          new TrackedTarget(
              Clock.time() - Units.millisecondsToSeconds(totalLatencyMs),
              cameraPose.getTranslation(),
              cameraPose.getRotation().getX() - Units.degreesToRadians(detections[i].txnc),
              cameraPose.getRotation().getY() + Units.degreesToRadians(detections[i].tync),
              detections[i].ta,
              -1,
              getPipeType());
    }

    return targets;
  }
}