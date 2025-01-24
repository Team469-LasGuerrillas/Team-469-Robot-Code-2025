package frc.lib.util.math.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.hardware.QuestNavUtil;
import frc.lib.util.math.GeomUtil;

public class VROdometry {
  private Pose2d m_poseMeters;
  private Transform2d robotToQuest;
  private QuestNavUtil questNav = QuestNavUtil.getInstance();

  public VROdometry(Pose2d initialPoseMeters, Transform2d robotToQuest) {
    m_poseMeters = initialPoseMeters;
    this.robotToQuest = robotToQuest;
    questNav.setPose(initialPoseMeters.plus(robotToQuest.inverse()));
  }

  public Pose2d getPoseMeters() {
    return m_poseMeters.plus(robotToQuest);
  }

  public void setPose(Pose2d pose) {
    questNav.setPose(pose);
    m_poseMeters = getPose();
  }

  public void setTranslation(Translation2d translation) {
    questNav.setPose(new Pose2d(translation, new Rotation2d(Math.toRadians(questNav.getOculusYaw()))));
    m_poseMeters = getPose();
  }

  public void setRotation(Rotation2d rotation) {
    questNav.setHeading(rotation);
    m_poseMeters = getPose();
  }

  public Pose2d update() {
    return getPoseMeters();
  }

  private Pose2d getPose() {
    return questNav.getPose();
  }
}
