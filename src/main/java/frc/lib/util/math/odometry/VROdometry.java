package frc.lib.util.math.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.hardware.QuestNavUtil;

public class VROdometry {
  QuestNavUtil questNav = QuestNavUtil.getInstance();

  Rotation2d questSpaceRotationOffset = new Rotation2d();
  Pose2d currentPose = new Pose2d();

  public VROdometry(Pose3d questPoseRobotSpace) {
    setRotation(new Rotation2d());
  }

  public void setRotation(Rotation2d rotationWPIBlue) {
    Rotation2d rotationQuestSpace = questNav.getPose().getRotation();

    questSpaceRotationOffset = rotationQuestSpace.plus(rotationWPIBlue);
  }

  public Pose2d update() {
    currentPose = questNav.getPose().rotateBy(questSpaceRotationOffset.unaryMinus());
    System.out.println("WCPCORGI: " + currentPose);
    return currentPose;
  }
}
