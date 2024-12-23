package frc.lib.interfaces.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.Clock;
import java.util.ArrayList;
import java.util.HashMap;
import org.photonvision.PhotonCamera;

public class VisionIOPhotonVision implements VisionIO {
  static HashMap<String, VisionIOPhotonVision> instances =
      new HashMap<String, VisionIOPhotonVision>();

  private final PhotonCamera camera;
  private final String cameraName;
  private final Transform3d robotToCam;

  private final int APRIL_TAG_PIPELINE_INDEX = 0;
  private final int OBJECT_DETECTION_PIPELINE_INDEX = 1;

  private boolean hasTargets = false;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCam) {
    camera = new PhotonCamera(cameraName);
    this.cameraName = cameraName;
    this.robotToCam = robotToCam;
  }

  public static VisionIOPhotonVision getInstance(String cameraName) {
    if (instances.get(cameraName) == null) {
      instances.put(cameraName, new VisionIOPhotonVision(cameraName, new Transform3d()));
    }
    return instances.get(cameraName);
  }

  @Override
  public void setPipelineIndex(int index) {
    camera.setPipelineIndex(index);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraPose = new Pose3d(robotToCam.getTranslation(), robotToCam.getRotation());
    inputs.isConnected = camera.isConnected();

    inputs.hasTargets = hasTargets;
    inputs.targettingType = getPipeType();

    inputs.targets = parseTargets();
    inputs.poseObservations = parsePoseObservations();
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

        for (var target : result.targets) {
          Pose3d cameraPose = new Pose3d(robotToCam.getTranslation(), robotToCam.getRotation());
          targets.add(
              new TrackedTarget(
                  Clock.time() - result.metadata.getLatencyMillis(),
                  cameraPose.getTranslation(),
                  cameraPose.getRotation().getX() - Units.degreesToRadians(target.yaw),
                  cameraPose.getRotation().getY() + Units.degreesToRadians(target.pitch),
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
        var multitagResult = result.multitagResult.get();

        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCam.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        poseObservations.add(
            new PoseObservation(
                Clock.time() - result.metadata.getLatencyMillis(), null, robotPose, null, null));
      }
    }

    return poseObservations.toArray(new PoseObservation[poseObservations.size()]);
  }
}
