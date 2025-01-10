package frc.lib.util.math.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.hardware.QuestNavUtil;

public class VROdometry {
  private Pose2d m_poseMeters;
  private QuestNavUtil questNav = QuestNavUtil.getInstance();

  public VROdometry(Pose2d initialPoseMeters) {
    m_poseMeters = initialPoseMeters;
  }

  public Pose2d getPoseMeters() {
    return m_poseMeters;
  }

  public void resetPose(Pose2d pose) {
    questNav.setPose(pose);
    m_poseMeters = getPose();
  }

  public void resetTranslation(Translation2d translation) {
    questNav.setPose(new Pose2d(translation, new Rotation2d(Math.toRadians(questNav.getOculusYaw()))));
    m_poseMeters = getPose();
  }

  public void setRotation(Rotation2d rotation) {
    questNav.setHeading(rotation);
    m_poseMeters = getPose();
  }

  public Pose2d update() {
    return getPose();
  }

  private Pose2d getPose() {
    return questNav.getPose();
  }
}
