package frc.lib.util.math.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.hardware.QuestNavUtil;
import frc.lib.util.math.GeomUtil;

public class VROdometry {
  private QuestNavUtil questNav = QuestNavUtil.getInstance();

  private Rotation2d questSpaceRotationOffset = new Rotation2d();
  private Pose3d questPoseRobotSpace = new Pose3d();

  Pose2d currentPose = new Pose2d();

  public VROdometry(Pose3d questPoseRobotSpace) {
    this.questPoseRobotSpace = questPoseRobotSpace;
    setQuestRotationRobotSpace(questPoseRobotSpace.getRotation().toRotation2d().plus(Rotation2d.fromDegrees(180)));
  }

  public void setRotationFieldSpace(Rotation2d fieldSpaceRotation) {
    setQuestRotationRobotSpace(fieldSpaceRotation.plus(Rotation2d.fromDegrees(180)));
  }

  public void setQuestRotationRobotSpace(Rotation2d robotRotation) {
    Rotation2d rotationQuestSpace = questNav.getPose().getRotation();

    questSpaceRotationOffset = rotationQuestSpace.minus(robotRotation);
  }

  public Pose2d update() {
    currentPose = questNav.getPose().plus(GeomUtil.toTransform2d(questPoseRobotSpace.toPose2d()).inverse()).rotateBy(questSpaceRotationOffset.unaryMinus());
    return currentPose;
  }
}
