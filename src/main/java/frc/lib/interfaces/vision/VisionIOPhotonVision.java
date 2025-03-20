package frc.lib.interfaces.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import frc.lib.util.Clock;
import frc.lib.util.math.GeomUtil;
import frc.lib.util.math.TrigEstimator;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

  List<PhotonPipelineResult> cameraUnreadResults;
  
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  PhotonPoseEstimator photonPoseEstimator;

  private VisionIOPhotonVision(String cameraName, Pose3d robotToCam) {
    camera = new PhotonCamera(cameraName);
    photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      GeomUtil.toTransform3d(robotToCam)
    );

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

    updateUnreadResults();

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

  private void updateUnreadResults() {
    cameraUnreadResults = camera.getAllUnreadResults();
  }

  private TrackedTarget[] parseTargets() {
    ArrayList<TrackedTarget> targets = new ArrayList<TrackedTarget>();

    for (var result : cameraUnreadResults) {
      if (result.hasTargets()) {
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
    photonPoseEstimator.setReferencePose(Drive.getInstance().getPose());

    for (var result : cameraUnreadResults) {
      var visionEst = photonPoseEstimator.update(result);

      if (visionEst.isPresent()) {
        hasTargets = true;

        var bestTarget = result.getBestTarget();

        Pose3d multitagPose = visionEst.get().estimatedPose;
        double timestamp = Clock.time() - Units.millisecondsToSeconds(result.metadata.getLatencyMillis());
        Pose3d tagPose = aprilTagFieldLayout.getTagPose(bestTarget.fiducialId).get();
        double tagCount = visionEst.get().targetsUsed.size();
        double distance3d = 
          multitagPose
            .plus(GeomUtil.toTransform3d(robotToCam.getSample(timestamp).get())).getTranslation()
            .getDistance(tagPose.getTranslation());

        poseObservations.add(
          new PoseObservation(
            timestamp,
            bestTarget.poseAmbiguity,
            bestTarget.area,
            visionEst.get().targetsUsed.size(),
            multitagPose,
            calculateSTDS(result, tagCount, PoseObservationType.MULTITAG_1),
            bestTarget.getFiducialId(),
            PoseObservationType.MULTITAG_1)
        );

        poseObservations.add(
          new PoseObservation(
            timestamp,
            0,
            0,
            visionEst.get().targetsUsed.size(),
            TrigEstimator.getTrigBasedEstimatedPose(
              distance3d, 
              bestTarget.yaw, 
              bestTarget.pitch, 
              timestamp, 
              robotToCam.getSample(timestamp).get(), 
              bestTarget.fiducialId
            ),
            calculateSTDS(result, tagCount, PoseObservationType.MULTITAG_2),
            bestTarget.getFiducialId(),
            PoseObservationType.MULTITAG_2)
        );

      }
    }
    return poseObservations.toArray(new PoseObservation[poseObservations.size()]);
  }

  private double[] calculateSTDS(PhotonPipelineResult result, double tagCount, PoseObservationType type) {
    double totalTagDistance = 0.0;
    for (var target : result.targets) {
      totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
    }

    double averageTagDistance = totalTagDistance / result.targets.size();

    double translationalStdBaseline = VisionConstants.MULTITAG_TRANSLATIONAL_FACTOR;;
    double rotationalStdBaseline = VisionConstants.MULTITAG_ROTATIONAL_FACTOR;

    if (type == PoseObservationType.MULTITAG_2) {
      translationalStdBaseline = VisionConstants.MULTITAG2_TRANSLATIONAL_FACTOR;
      rotationalStdBaseline = VisionConstants.MULITAG2_ROTATIONAL_FACTOR;
    }

    double stdDevFactor = Math.pow(averageTagDistance, 2) / tagCount;
    double linearStdDev = translationalStdBaseline * stdDevFactor;
    double angularStdDev = rotationalStdBaseline * stdDevFactor;

    return new double[] {linearStdDev, linearStdDev, angularStdDev};
  }
}