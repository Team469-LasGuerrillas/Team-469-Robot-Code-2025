package frc.lib.util.math.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.hardware.QuestNavUtil;

public class VROdometry {
  private Pose2d m_poseMeters;
  private Pose2d robotToQuest;
  private QuestNavUtil questNav = QuestNavUtil.getInstance();

  public VROdometry(Pose2d initialPoseMeters, Pose2d robotToQuest, Rotation2d initHeading) {
    m_poseMeters = initialPoseMeters;
    this.robotToQuest = robotToQuest;
    initHeading(initHeading);
  }

  public void initHeading(Rotation2d initRotation) {
    questNav.initHeading((float) initRotation.getDegrees());
  }

  public void setRotation(Rotation2d rotation) {
    questNav.zeroHeading((float) (rotation.getDegrees()));
  }

  public Pose2d update() {
    return questNav.getPose();
  }
}
