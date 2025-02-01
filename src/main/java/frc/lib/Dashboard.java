package frc.lib;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Dashboard {
  public static final Field2d m_field = new Field2d();
  private static SendableChooser<Command> autoChooser;

  private static boolean isCompetition = false;

  public static void addWidgets(ShuffleboardTab tab) {
    tab
    .add(m_field)
    .withSize(6, 6)
    .withPosition(0, 0);

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    tab
    .add("Auton Chooser", autoChooser)
    .withSize(3, 1)
    .withPosition(7, 0);
  }

  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
