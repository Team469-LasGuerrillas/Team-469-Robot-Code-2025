package frc.lib.util.math.estimator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.util.Clock;
import frc.lib.util.math.InterpolatorUtil;
import frc.lib.util.math.odometry.OdometryType;
import frc.lib.util.math.odometry.VROdometry;
import frc.robot.subsystems.Constants.VisionConstants;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

public class PoseSequencingEstimator<T> {
  private final Odometry<T> primaryOdometry;
  private final VROdometry secondaryOdometry;

  private final Matrix<N3, N1> q = new Matrix<>(Nat.N3(), Nat.N1());
  private final Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());

  private static final double bufferDuration = 2; // Seconds
  private final TimeInterpolatableBuffer<Pose2d> odometryBuffer =
      TimeInterpolatableBuffer.createBuffer(bufferDuration);

  private final NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

  private Pose2d currentPoseEstimate;

  private Pose2d lastPrimaryOdometryPose;
  private Pose2d lastSecondaryOdometryPose;

  private OdometryType odometryType;

  private int numberOfRecursions = 0;

  @SuppressWarnings("PMD.UnusedxFormalParameter")
  public PoseSequencingEstimator(
      Kinematics<?, T> kinematics,
      Odometry<T> primaryOdometry,
      VROdometry secondaryOdometry,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs,
      OdometryType odometryType) {
    this.primaryOdometry = primaryOdometry;
    this.secondaryOdometry = secondaryOdometry;
    this.odometryType = odometryType;

    currentPoseEstimate = primaryOdometry.getPoseMeters();

    lastPrimaryOdometryPose = currentPoseEstimate;
    lastSecondaryOdometryPose = currentPoseEstimate;
    odometryBuffer.addSample(Clock.time(), currentPoseEstimate);

    for (int i = 0; i < 3; ++i) {
      q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stds) {

    if (odometryBuffer.getInternalBuffer().isEmpty()
        || odometryBuffer.getInternalBuffer().firstKey() > timestamp) {
      return;
    }

    cleanUpVisionUpdates();

    var odometrySample = odometryBuffer.getSample(timestamp);

    if (odometrySample.isEmpty()) {
      return;
    }

    var estimatedSample = getPoseEstimate(timestamp);

    if (estimatedSample.isEmpty()) {
      return;
    }

    setVisionMeasurementStdDevs(stds);

    var twist = estimatedSample.get().log(pose);

    var kalmanFactor = visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

    var kalmanFactorTwist =
        new Twist2d(kalmanFactor.get(0, 0), kalmanFactor.get(1, 0), kalmanFactor.get(2, 0));

    var visionUpdate =
        new VisionUpdate(
            pose, estimatedSample.get().exp(kalmanFactorTwist), odometrySample.get(), stds);
    visionUpdates.put(timestamp, visionUpdate);

    // Replay pose estimates that are timestamped earlier
    var timeStampToReplay = searchForNextNewestUpdate(timestamp);

    if (timeStampToReplay.isPresent()) {
      numberOfRecursions++;

      VisionUpdate visionUpdateToReplay = visionUpdates.get(timeStampToReplay.get());
      Pose2d rawPoseToReplay = visionUpdateToReplay.getRawVisionPose();
      Matrix<N3, N1> stdsToReplay = visionUpdateToReplay.getStds();

      visionUpdates.remove(timeStampToReplay.get());
      System.out.println(numberOfRecursions);

      addVisionMeasurement(rawPoseToReplay, timeStampToReplay.get(), stdsToReplay);
    } else {
      numberOfRecursions = 0;
      System.out.println(numberOfRecursions);
    }
  }

  private Optional<Double> searchForNextNewestUpdate(double timestamp) {
    // Check to make sure there is another value that occured later
    if (visionUpdates.isEmpty() || timestamp == visionUpdates.ceilingKey(timestamp)) {
      return Optional.empty();
    }

    // Get the timestamp of the next update
    double nextNewestTimestamp = visionUpdates.ceilingKey(timestamp);
    return Optional.of(nextNewestTimestamp);
  }

  public void resetPosition(Rotation2d gyroAngle, T wheelPositions, Pose2d poseMeters) {
    // Reset state estimate and error covariance
    primaryOdometry.resetPosition(gyroAngle, wheelPositions, poseMeters);
    secondaryOdometry.resetPose(poseMeters);
    lastPrimaryOdometryPose = poseMeters;
    lastSecondaryOdometryPose = poseMeters;
    odometryBuffer.clear();
    visionUpdates.clear();
    currentPoseEstimate = primaryOdometry.getPoseMeters();

    odometryBuffer.addSample(Clock.time(), currentPoseEstimate);
  }

  public void resetPose(Pose2d pose) {
    primaryOdometry.resetPose(pose);
    secondaryOdometry.resetPose(pose);
    lastPrimaryOdometryPose = pose;
    lastSecondaryOdometryPose = pose;
    odometryBuffer.clear();
    visionUpdates.clear();
    currentPoseEstimate = primaryOdometry.getPoseMeters();

    odometryBuffer.addSample(Clock.time(), currentPoseEstimate);
  }

  public void resetTranslation(Translation2d translation) {
    primaryOdometry.resetTranslation(translation);
    secondaryOdometry.resetTranslation(translation);
    lastPrimaryOdometryPose = new Pose2d(translation, lastPrimaryOdometryPose.getRotation());
    lastSecondaryOdometryPose = new Pose2d(translation, lastSecondaryOdometryPose.getRotation());
    odometryBuffer.clear();
    visionUpdates.clear();
    currentPoseEstimate = primaryOdometry.getPoseMeters();

    odometryBuffer.addSample(Clock.time(), currentPoseEstimate);
  }

  public void resetRotation(Rotation2d rotation) {
    primaryOdometry.resetRotation(rotation);
    secondaryOdometry.setRotation(rotation);
    lastPrimaryOdometryPose = new Pose2d(lastPrimaryOdometryPose.getTranslation(), rotation);
    lastSecondaryOdometryPose = new Pose2d(lastSecondaryOdometryPose.getTranslation(), rotation);
    odometryBuffer.clear();
    visionUpdates.clear();
    currentPoseEstimate = primaryOdometry.getPoseMeters();

    odometryBuffer.addSample(Clock.time(), currentPoseEstimate);
  }

  public Pose2d getCurrentPoseEstimate() {
    if (currentPoseEstimate != null) {
      return currentPoseEstimate;
    } else {
      return new Pose2d();
    }
  }

  public Pose2d getRawPose() {
    return primaryOdometry.getPoseMeters();
  }

  public Optional<Pose2d> getPoseEstimate(double timestamp) {
    if (odometryBuffer.getInternalBuffer().isEmpty()) {
      return Optional.empty();
    }

    double oldestOdometryTimestamp = odometryBuffer.getInternalBuffer().firstKey();
    double newestOdometryTimestamp = odometryBuffer.getInternalBuffer().lastKey();

    if (timestamp < oldestOdometryTimestamp) {
      // TODO: implement legacy buffer
    }

    timestamp = MathUtil.clamp(timestamp, oldestOdometryTimestamp, newestOdometryTimestamp);

    if (visionUpdates.isEmpty() || timestamp < visionUpdates.firstKey()) {
      return odometryBuffer.getSample(timestamp);
    }

    double floorTimestamp = visionUpdates.floorKey(timestamp);
    var visionUpdate = visionUpdates.get(floorTimestamp);

    var odometryEstimate = odometryBuffer.getSample(timestamp);

    // Step 5: Apply the vision compensation to the odometry pose.
    return odometryEstimate.map(odometryPose -> visionUpdate.compensate(odometryPose));
  }

  private void cleanUpVisionUpdates() {
    // Step 0: If there are no odometry samples, skip.
    if (odometryBuffer.getInternalBuffer().isEmpty()) {
      return;
    }

    // Step 1: Find the oldest timestamp that needs a vision update.
    double oldestOdometryTimestamp = odometryBuffer.getInternalBuffer().firstKey();

    // Step 2: If there are no vision updates before that timestamp, skip.
    if (visionUpdates.isEmpty() || oldestOdometryTimestamp <= visionUpdates.firstKey()) {
      return;
    }

    // Step 3: Find the newest vision update timestamp before or at the oldest timestamp.
    double newestNeededVisionUpdateTimestamp = visionUpdates.floorKey(oldestOdometryTimestamp);

    // Step 4: Remove all entries strictly before the newest timestamp we need.
    visionUpdates.headMap(newestNeededVisionUpdateTimestamp, true).clear();
  }

  public Pose2d updateWithTime(
      double currentTime,
      Rotation2d gyroAngle,
      T wheelPositions,
      double secondaryTrust,
      boolean secondaryConnected) {
    var primaryOdometryEstimate = primaryOdometry.update(gyroAngle, wheelPositions);
    var secondaryOdometryEstimate = secondaryOdometry.update();

    Transform2d primaryDelta = primaryOdometryEstimate.minus(lastPrimaryOdometryPose);
    Transform2d secondaryDelta = secondaryOdometryEstimate.minus(lastSecondaryOdometryPose);
    Transform2d interpolatedDelta;

    if (odometryType == OdometryType.FUSED_ODOMETRY && secondaryConnected) {
      interpolatedDelta = InterpolatorUtil.transform2d(primaryDelta, secondaryDelta, secondaryTrust); // Fused Odometry
    } else if (odometryType == OdometryType.WHEEL_ODOMETRY) {
      interpolatedDelta = primaryDelta; // Wheel Odometry
    } else {
      interpolatedDelta = secondaryDelta; // VR Odometry
    }

    Pose2d interpolatedEstimate =
        odometryBuffer.getInternalBuffer().lastEntry().getValue().plus(interpolatedDelta);

    odometryBuffer.addSample(currentTime, interpolatedEstimate);

    if (visionUpdates.isEmpty()) {
      currentPoseEstimate = interpolatedEstimate;
    } else {
      var visionUpdate = visionUpdates.get(visionUpdates.lastKey());
      currentPoseEstimate = visionUpdate.compensate(interpolatedEstimate);
    }

    lastPrimaryOdometryPose = primaryOdometryEstimate;
    lastSecondaryOdometryPose = secondaryOdometryEstimate;

    return getCurrentPoseEstimate();
  }

  public final void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    for (int row = 0; row < 3; ++row) {
      if (q.get(row, 0) == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row])));
      }
    }
  }

  private static final class VisionUpdate {
    // THe absolute measured vision pose
    private final Pose2d rawVisionPose;

    // The vision-compensated pose estimate.
    private final Pose2d filteredVisionPose;

    // The pose estimated based solely on odometry.
    private final Pose2d odometryPose;

    // The stds
    private final Matrix<N3, N1> stds;

    /**
     * Constructs a vision update record with the specified parameters.
     *
     * @param filteredVisionPose The vision-compensated pose estimate.
     * @param odometryPose The pose estimate based solely on odometry.
     */
    private VisionUpdate(
        Pose2d rawVisionPose, Pose2d filteredVisionPose, Pose2d odometryPose, Matrix<N3, N1> stds) {
      this.rawVisionPose = rawVisionPose;
      this.filteredVisionPose = filteredVisionPose;
      this.odometryPose = odometryPose;
      this.stds = stds;
    }

    /**
     * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
     * relative to this record's odometry pose to being relative to this record's vision pose.
     *
     * @param pose The pose to compensate.
     * @return The compensated pose.
     */
    public Pose2d compensate(Pose2d pose) {
      var delta = pose.minus(this.odometryPose);
      return this.filteredVisionPose.plus(delta);
    }

    public Pose2d getRawVisionPose() {
      return rawVisionPose;
    }

    public Matrix<N3, N1> getStds() {
      return stds;
    }
  }
}
