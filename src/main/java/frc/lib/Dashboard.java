package frc.lib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autons.Autons;

public class Dashboard {
  public static final Field2d m_field = new Field2d();
  public static final Field2d m_visionField = new Field2d();

  private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private static boolean isCompetition = false;

  public static void addWidgets(ShuffleboardTab tab) {
    tab
    .add("Good Odometry", m_field)
    .withSize(7, 6)
    .withPosition(0, 0);

    tab
    .add("Vision Odometry", m_visionField)
    .withSize(6, 6)
    .withPosition(7, 0);

    // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    //   (stream) -> isCompetition
    //     ? stream.filter(auto -> auto.getName().startsWith("comp"))
    //     : stream
    // );

    autoChooser.setDefaultOption("1 Piece Center", Autons.centerOnePiece());
    autoChooser.addOption("1 Piece + Algae", Autons.centerOnePiecePlusAlgae());
    autoChooser.addOption("3 Piece Left", Autons.threePieceLeft());
    autoChooser.addOption("3 Piece Right", Autons.threePieceRight());

    tab
    .add("Auton Chooser", autoChooser)
    .withSize(2, 5)
    .withPosition(13, 0);
  }

  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
