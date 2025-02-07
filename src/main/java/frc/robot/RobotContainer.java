// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Dashboard;
import frc.lib.interfaces.vision.VisionIO;
import frc.robot.commandfactories.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final Vision limelightLeft;
  // private final Vision limelightRight;
  private final Vision arducamOne;
  private final Vision arducamTwo;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drive Station 2025");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        Drive.createInstance(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
        drive = Drive.getInstance();

        limelightLeft = new Vision(VisionConstants.LIMELIGHT_LEFT);
        // limelightRight = new Vision(VisionConstants.LIMELIGHT_RIGHT);
        arducamOne = new Vision(VisionConstants.ARDUCAM_ONE);
        arducamTwo = new Vision(VisionConstants.ARDUCAM_TWO);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        Drive.createInstance(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
        drive = Drive.getInstance();

        limelightLeft = new Vision(new VisionIO() {});
        // limelightRight = new Vision(new VisionIO() {});
        arducamOne = new Vision(new VisionIO() {});
        arducamTwo = new Vision(new VisionIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        Drive.createInstance(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        drive = Drive.getInstance();

        limelightLeft = new Vision(new VisionIO() {});
        // limelightRight = new Vision(new VisionIO() {});
        arducamOne = new Vision(new VisionIO() {});
        arducamTwo = new Vision(new VisionIO() {});

        break;
    }

    // Add Dashboard Widgets
    Dashboard.addWidgets(shuffleboardTab);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.acceptTeleopFieldOriented(controller, true));

    controller.x().whileTrue(DriveCommands.autoRotate(controller, () -> new Rotation2d()));
 
    // controller.y().whileTrue(DriveCommands.aimAssistToPose(controller, new Pose2d(2, 4.03, new Rotation2d()))
    // );

    // controller.y().whileTrue(
    //   DriveCommands.pidToClosestReefPose()
    // );

    controller.rightBumper().whileTrue(
      DriveCommands.aimAssistToClosestReefPose(controller, 0.469)
    );

    controller.y().whileTrue(DriveCommands.pidToClosestReefPose());
    
    controller.a().whileTrue(Commands.startEnd(
      () -> drive.setPathfinding(new Pose2d(2, 4.05, new Rotation2d())), 
      () -> drive.clearMode())
    );

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset Pose when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d()),
                    drive)
                .ignoringDisable(true));
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Dashboard.getAutonomousCommand();
  }
}