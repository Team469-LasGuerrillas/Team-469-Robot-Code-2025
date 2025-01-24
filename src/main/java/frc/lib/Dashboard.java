package frc.lib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Dashboard {
  public static final Field2d m_field = new Field2d();

  public static void addWidgets(ShuffleboardTab tab) {
    tab
    .add(m_field)
    .withSize(13, 6)
    .withPosition(0, 0);
  }
}
