package frc.lib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Dashboard {
  public static final Field2d m_field = new Field2d();

  public static void addWidgets(ShuffleboardTab tab) {
    tab
    .add(m_field)
    .withPosition(0, 0)
    .withSize(5, 5);
  }
}
