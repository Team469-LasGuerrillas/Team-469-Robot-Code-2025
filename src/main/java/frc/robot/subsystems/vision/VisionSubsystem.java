package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.vision.VisionIO;
import frc.lib.interfaces.vision.VisionIO.PoseObservation;
import frc.lib.interfaces.vision.VisionIO.PoseObservationType;
import frc.lib.interfaces.vision.VisionIO.TargettingType;
import frc.robot.subsystems.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.lib.interfaces.vision.VisionIOInputsAutoLogged;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getCameraName(), inputs);

    List<PoseObservation> robotPosesAccepted = new LinkedList<>();
    List<PoseObservation> robotPosesRejected = new LinkedList<>();

    if ((hasLatestUpdate() && getTargettingType() == TargettingType.FIDUCIAL) && hasTargets()) {
      for (PoseObservation observation : inputs.poseObservations) {
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > VisionConstants.MAX_Z_ERROR // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX()
                    > 10 // TODO: USE APRIL TAG FIELD LAYOUT WITH NEW VENDORDEP
                || observation.pose().getY() < 0.0
                || observation.pose().getY()
                    > 10; // TODO: USE APRIL TAG FIELD LAYOUT WITH NEW VENDORDEP

        if (rejectPose) {
          robotPosesRejected.add(observation);
          System.out.println("varun tandoori");
        } else {
          robotPosesAccepted.add(observation);

          System.out.println("om joshi");


          boolean updateYaw =
              observation.tagCount() >= 2
                  && (observation.type() == PoseObservationType.MEGATAG_1
                      || observation.type() == PoseObservationType.MULTITAG);

          Drive.getInstance()
              .addVisionMeasurement(
                  observation.pose().toPose2d(),
                  observation.timestamp(),
                  VecBuilder.fill(
                      observation.stdDevs()[0],
                      observation.stdDevs()[1],
                      updateYaw ? observation.stdDevs()[2] : Double.POSITIVE_INFINITY));
        }

        Logger.recordOutput(
            getCameraName() + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new PoseObservation[robotPosesAccepted.size()]));

        Logger.recordOutput(
            getCameraName() + "/RobotPosesRejected",
            robotPosesRejected.toArray(new PoseObservation[robotPosesRejected.size()]));
      }
    }
  }

  public String getCameraName() {
    return inputs.cameraName;
  }

  public boolean hasLatestUpdate() {
    return inputs.hasLatestUpdate;
  }

  public TargettingType getTargettingType() {
    return inputs.targettingType;
  }

  public void setCamerPose(Pose3d cameraPose) {
    io.setPoseRobotSpace(cameraPose);
  }

  public boolean hasTargets() {
    return inputs.hasTargets;
  }

  // TODO: Add other functions for fetching Camera Data Later
}
