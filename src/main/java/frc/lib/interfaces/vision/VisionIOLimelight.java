package frc.lib.interfaces.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.Clock;
import frc.robot.subsystems.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawDetection;
import frc.robot.util.LimelightHelpers.RawFiducial;

import java.util.HashMap;

/*
                  .:------:.
              .-===-:::::--=+=-.
            :=+-.            .-+=:
         .=+:    -+**##**=.    :=+:
       .=+.   .:-:    :+#####.      =+.
     :+-   .#########*    *##: :%*    -+:
  .-=+.    *####+-.       .#*  -##=    :+=-:
+=        -%*:  .:            =####.        =+
+-        -#. .+#*           -####*         -+
  -=+=.     .####.        =####*=.     .=+=-
     =+:    -####:  *+:    ..         :+-
      :+-   :####=  =%#*=-:..:-=*:   :+:
       .+=   =####.  *########%#-   -+:
          :+=.  :+##.  -=++=-   .-+-
              :=+=-:......:-=+=:
                 .:-======-:.
*/

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
  private VisionIOLimelight(String limelightName, Pose3d cameraPose) {
    this.limelightName = limelightName;

    fiducials = LimelightHelpers.getRawFiducials(limelightName);
    detections = LimelightHelpers.getRawDetections(limelightName);

    setPoseRobotSpace(cameraPose);
  }

  public static VisionIOLimelight getInstance(String limelightName, Pose3d cameraPose) {
    if (instances.get(limelightName) == null) {
      instances.put(limelightName, new VisionIOLimelight(limelightName, cameraPose));
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
    inputs.poseObservations = parsePoseObservations();

    double[] hw = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "hw");
    if(hw.length != 0){
      inputs.fps = hw[0];
      inputs.cpuTemp = hw[1];
      inputs.ram = hw[2];
    }

    inputs.cameraName = limelightName;
  }

  @Override
  public void setPoseRobotSpace(Pose3d pose) {
    LimelightHelpers.setCameraPose_RobotSpace(
        limelightName,
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        Math.toDegrees(pose.getRotation().getX()),
        Math.toDegrees(pose.getRotation().getY()),
        Math.toDegrees(pose.getRotation().getZ())
    );
  }

  @Override
  public void setRobotRotationUpdate(Rotation2d rotation, Rotation2d angularVelocity) {
    LimelightHelpers.SetRobotOrientation(
        limelightName,
        rotation.getDegrees(),
        angularVelocity.getDegrees(),
        0,
        0,
        0,
        0);
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

  private PoseObservation[] parsePoseObservations() {
    double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "stddevs");
    RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightName);

    if (stddevs.length == 0 || rawFiducials.length == 0) {
      return new PoseObservation[] {};
    }
    
    PoseObservation[] poses = {
      new PoseObservation(
          Clock.time() - Units.millisecondsToSeconds(totalLatencyMs),
          rawFiducials[0].ambiguity, // only applicable for 1 tag
          LimelightHelpers.getTA(limelightName),
          LimelightHelpers.getTargetCount(limelightName),
          LimelightHelpers.getBotPose3d_wpiBlue_MegaTag2(limelightName),
          new double[] {stddevs[0] * VisionConstants.MT_TRANSLATIONAL_FACTOR, stddevs[1] * VisionConstants.MT_TRANSLATIONAL_FACTOR, stddevs[5] * VisionConstants.MT_ROTATIONAL_FACTOR},
          rawFiducials[0].id,
          PoseObservationType.MEGATAG_1),
      new PoseObservation(
          Clock.time() - Units.millisecondsToSeconds(totalLatencyMs + 0.001),
          rawFiducials[0].ambiguity, // only applicable for 1 tag
          LimelightHelpers.getTA(limelightName),
          LimelightHelpers.getTargetCount(limelightName),
          LimelightHelpers.getBotPose3d_wpiBlue_MegaTag2(limelightName),
          new double[] {stddevs[6] * VisionConstants.MT2_TRANSLATIONAL_FACTOR, stddevs[7] * VisionConstants.MT2_TRANSLATIONAL_FACTOR, stddevs[11] * VisionConstants.MT2_ROTATIONAL_FACTOR},
          rawFiducials[0].id,
          PoseObservationType.MEGATAG_2)
    };

    return poses;
  }
}
