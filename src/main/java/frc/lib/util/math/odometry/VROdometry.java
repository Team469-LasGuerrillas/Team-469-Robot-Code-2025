package frc.lib.util.math.odometry;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public void setPose(Pose2d pose) {
    QuestNavUtil.getInstance().zeroPosition();
    m_poseMeters = getPose();
  }

  public void setRotation(Rotation2d rotation) {
    questNav.zeroHeading((float) (rotation.getDegrees()));
    m_poseMeters = getPose();
  }

  public Pose2d update() {
    return getPose();
  }

  private Pose2d getPose() {
    return questNav.getPose();
  }
}
