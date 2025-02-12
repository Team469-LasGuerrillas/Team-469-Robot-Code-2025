package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Dashboard;
import frc.lib.interfaces.vision.VisionIO;
import frc.lib.interfaces.vision.VisionIO.PoseObservation;
import frc.lib.interfaces.vision.VisionIO.PoseObservationType;
import frc.lib.interfaces.vision.VisionIO.TargettingType;
import frc.lib.util.Clock;
import frc.lib.util.math.GeomUtil;
import frc.lib.util.math.ToleranceUtil;
import frc.robot.subsystems.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.lib.interfaces.vision.VisionIOInputsAutoLogged;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  
  // Camera Update / OnlyReefUpdates Boolean
  public static String onlyReefUpdateCamera = "";
  public static boolean onlyReefUpdateGlobal = false;
  public static double lastGoodReefUpdateTime = 0;

  public Vision(VisionIO io) {
    this.io = io;
    io.setTagFiltersOverride(VisionConstants.REEF_TAG_IDS);
  }
  
  @Override
  public void periodic() {
    setRobotYaw();
    io.updateInputs(inputs);
    Logger.processInputs(getCameraName(), inputs);

    List<PoseObservation> robotPosesAccepted = new LinkedList<>();
    List<PoseObservation> robotPosesRejected = new LinkedList<>();
    
    boolean isOldReefOnlyTime = Clock.time() - lastGoodReefUpdateTime > VisionConstants.LAST_GOOD_UPDATE_TIME_THRESHOLD;

    if (isOldReefOnlyTime) {
      onlyReefUpdateGlobal = false;
      io.setTagFiltersOverride(VisionConstants.ALL_TAG_IDS);
    }

    if ((hasLatestUpdate() && getTargettingType() == TargettingType.FIDUCIAL) && hasTargets()) {
      for (PoseObservation observation : inputs.poseObservations) {
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) // Cannot be high ambiguity
                || (observation.tagCount() == 1
                    && ((observation.type() == PoseObservationType.MEGATAG_1 && observation.ta() < VisionConstants.MAX_MEGATAG_SINGLE_TA)
                    || (observation.type() == PoseObservationType.MULTITAG_1 && observation.ta() < VisionConstants.MAX_MULITAG_SINGLE_TA)))
                || ((observation.type() == PoseObservationType.MULTITAG_2 || observation.type() == PoseObservationType.MEGATAG_2) && !ToleranceUtil.epsilonEquals(observation.pose().getRotation().toRotation2d().getDegrees(), Drive.getInstance().getRotation().getDegrees(), 1))
                || Math.abs(observation.pose().getZ())
                    > VisionConstants.MAX_Z_ERROR // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() <= 0.0
                || observation.pose().getX()
                    >= Units.feetToMeters(26)  
                || observation.pose().getY() <= 0.0
                || observation.pose().getY()
                    >= Units.feetToMeters(57) 
                || Math.abs(Drive.getInstance().getRobotRelativeVelocity().dtheta) > VisionConstants.MAX_YAW_RATE;

        boolean onlyReefUpdateLocal = 
          GeomUtil.isLookingAtReef() 
          && GeomUtil.isWithinReefRadius() 
          && (observation.type() == PoseObservationType.MEGATAG_1 
              || observation.type() == PoseObservationType.MEGATAG_2);

          // TODO: && IF WE CURRENTLY HAVE A CORAL
        
        if (onlyReefUpdateLocal) {
          onlyReefUpdateGlobal = true;
          onlyReefUpdateCamera = getCameraName();
          lastGoodReefUpdateTime = Clock.time();
        }

        if (onlyReefUpdateGlobal) {
          io.setTagFiltersOverride(VisionConstants.REEF_TAG_IDS);

          if (observation.type() == PoseObservationType.MULTITAG_1 || observation.type() != PoseObservationType.MULTITAG_2) {
            rejectPose = true;
          }
        } else if (!onlyReefUpdateGlobal || (!onlyReefUpdateLocal && onlyReefUpdateCamera.equals(getCameraName()))) {
          onlyReefUpdateGlobal = false;
          io.setTagFiltersOverride(VisionConstants.ALL_TAG_IDS);
        }

        Dashboard.m_visionField.setRobotPose(observation.pose().toPose2d());

        Logger.recordOutput(getCameraName(), !rejectPose);

        if (rejectPose) {
          robotPosesRejected.add(observation);
          System.out.println("REJECTING!!! " + getCameraName() + "Translation: " + observation.pose().getTranslation() + " Tag Type: " + observation.type() + " Tag Count: " + observation.tagCount() + " ambiguity: " + observation.ambiguity() + " z: " + observation.pose().getZ() + " TA: " + observation.ta());
          // System.out.println("REJECTING!!! Tag Type: " + observation.type());
        } else {
          robotPosesAccepted.add(observation);
          System.out.println("ACCEPTING!!! " + getCameraName() + " Tag Type: " + observation.type() + " Tag Count: " + observation.tagCount() + " ambiguity: " + observation.ambiguity() + " z: " + observation.pose().getZ() + "STDS: " + observation.stdDevs());
          
          boolean updateYaw =
              observation.tagCount() >= 2
                && (observation.type() == PoseObservationType.MEGATAG_1 || observation.type() == PoseObservationType.MULTITAG_1);
            
          if (updateYaw) {
            System.out.println("Updating Yaw with " + getCameraName() + "STD: " + observation.stdDevs()[2] + " Tag Count: " + observation.tagCount());
          }

          Drive.getInstance()
              .addVisionMeasurement(
                  observation.pose().toPose2d(),
                  observation.timestamp(),
                  VecBuilder.fill(
                      observation.stdDevs()[0],
                      observation.stdDevs()[1],
                      updateYaw ? observation.stdDevs()[2] : VisionConstants.MEGATAG2_ROTATIONAL_FACTOR));
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

  public void setRobotYaw() {
    io.setRobotRotationUpdate(Drive.getInstance().getRotation(), new Rotation2d(Drive.getInstance().getRobotRelativeVelocity().dtheta));
  }

  public boolean isReefId(int id) {
    if((id >= 17 && id <= 22) || (id >= 6 && id <= 11)) {
      return true;
    }
    return false;
  }

}
