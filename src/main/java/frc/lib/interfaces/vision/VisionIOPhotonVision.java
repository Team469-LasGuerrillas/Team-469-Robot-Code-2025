package frc.lib.interfaces.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import frc.lib.util.Clock;
import frc.lib.util.math.GeomUtil;
import frc.robot.subsystems.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

  List<PhotonPipelineResult> cameraUnreadResults;
  
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  PhotonPoseEstimator photonPoseEstimator;

  private VisionIOPhotonVision(String cameraName, Pose3d robotToCam) {
    camera = new PhotonCamera(cameraName);
    photonPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, GeomUtil.toTransform3d(robotToCam));

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
    
    for (var result : cameraUnreadResults) {
      double tagArea = 0;
      if (result.hasTargets()) {
       tagArea = result.getBestTarget().area;
      }

      var visionEst = photonPoseEstimator.update(result);

      if (visionEst.isPresent()) {
        System.out.println(visionEst.get().estimatedPose);
      }

      if (result.multitagResult.isPresent()) {
        System.out.println("HOLA COMO ESTAS AMIGOS JAJAJAJAJAJAJAJAJA");
        hasTargets = true;

        var multitagResult = result.multitagResult.get();
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
                tagArea,
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