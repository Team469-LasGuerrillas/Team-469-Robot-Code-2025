package frc.lib.interfaces.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import frc.frc2025.subsystems.Constants.VisionConstants;
import frc.lib.util.Clock;
import java.util.ArrayList;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {
  static HashMap<String, VisionIOPhotonVision> instances =
      new HashMap<String, VisionIOPhotonVision>();

  private final PhotonCamera camera;
  private final String cameraName;
  private final TimeInterpolatableBuffer<Pose3d> robotToCam =
      TimeInterpolatableBuffer.createBuffer(2);

  private final int APRIL_TAG_PIPELINE_INDEX = 0;
  private final int OBJECT_DETECTION_PIPELINE_INDEX = 1;

  private boolean hasTargets = false;

  private VisionIOPhotonVision(String cameraName, Pose3d robotToCam) {
    camera = new PhotonCamera(cameraName);
    this.cameraName = cameraName;
    this.robotToCam.addSample(Clock.time(), robotToCam);
  }

  public static VisionIOPhotonVision getInstance(String cameraName, Pose3d robotToCam) {
    if (instances.get(cameraName) == null) {
      instances.put(cameraName, new VisionIOPhotonVision(cameraName, robotToCam));
    }
    return instances.get(cameraName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraPose = robotToCam.getSample(Clock.time()).get();
    inputs.hasLatestUpdate = camera.isConnected();

    inputs.hasTargets = hasTargets;
    inputs.targettingType = getPipeType();

    inputs.targets = parseTargets();
    inputs.poseObservations = parsePoseObservations();

    inputs.cameraName = cameraName;
  }

  @Override
  public void setPoseRobotSpace(Pose3d cameraPose) {
    robotToCam.addSample(Clock.time(), cameraPose);
  }

  @Override
  public void setPipelineIndex(int index) {
    camera.setPipelineIndex(index);
  }

  private TargettingType getPipeType() {
    int index = camera.getPipelineIndex();

    TargettingType type;

    switch (index) {
      case APRIL_TAG_PIPELINE_INDEX:
        type = TargettingType.FIDUCIAL;
        break;
      case OBJECT_DETECTION_PIPELINE_INDEX:
        type = TargettingType.OBJECT_DETECTION;
      default:
        type = TargettingType.FIDUCIAL;
        break;
    }

    return type;
  }

  private TrackedTarget[] parseTargets() {
    ArrayList<TrackedTarget> targets = new ArrayList<TrackedTarget>();

    for (var result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        hasTargets = true;
        double latency = Units.millisecondsToSeconds(result.metadata.getLatencyMillis());
        Pose3d instantaneousCameraPose = robotToCam.getSample(Clock.time()).get();

        for (var target : result.targets) {
          targets.add(
              new TrackedTarget(
                  Clock.time() - latency,
                  instantaneousCameraPose.getTranslation(),
                  instantaneousCameraPose.getRotation().getX() - Units.degreesToRadians(target.yaw),
                  instantaneousCameraPose.getRotation().getY()
                      + Units.degreesToRadians(target.pitch),
                  target.area,
                  target.fiducialId,
                  getPipeType()));
        }
      }
    }

    return targets.toArray(new TrackedTarget[targets.size()]);
  }

  private PoseObservation[] parsePoseObservations() {
    ArrayList<PoseObservation> poseObservations = new ArrayList<PoseObservation>();

    for (var result : camera.getAllUnreadResults()) {
      if (result.multitagResult.isPresent()) {
        MultiTargetPNPResult multitagResult = result.multitagResult.get();
        double latency = Units.millisecondsToSeconds(result.metadata.getLatencyMillis());
        Pose3d instantaneousCameraPose = robotToCam.getSample(Clock.time()).get();

        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot =
            fieldToCamera.plus(new Transform3d(new Pose3d(), instantaneousCameraPose).inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        poseObservations.add(
            new PoseObservation(
                Clock.time() - latency,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                robotPose,
                calculateSTDS(result, multitagResult),
                PoseObservationType.MULTITAG));
      }
    }

    return poseObservations.toArray(new PoseObservation[poseObservations.size()]);
  }

  private double[] calculateSTDS(PhotonPipelineResult result, MultiTargetPNPResult multitagResult) {
    double totalTagDistance = 0.0;
    for (var target : result.targets) {
      totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
    }

    double averageTagDistance = totalTagDistance / result.targets.size();
    double tagCount = multitagResult.fiducialIDsUsed.size();

    double stdDevFactor = Math.pow(averageTagDistance, 2) / tagCount;
    double linearStdDev = VisionConstants.LINEAR_STD_BASELINE * stdDevFactor;
    double angularStdDev = VisionConstants.ANGULAR_STD_BASELINE * stdDevFactor;

    return new double[] {linearStdDev, linearStdDev, angularStdDev};
  }
}
