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

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Dashboard;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.sensor.SensorIO;
import frc.lib.interfaces.vision.VisionIO;
import frc.lib.util.FieldLayout;
import frc.robot.autons.Autons;
import frc.robot.commandfactories.AutonCommands;
import frc.robot.commandfactories.DriveCommands;
import frc.robot.commandfactories.ElevatorCommands;
import frc.robot.commandfactories.GlobalCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.constants.AlgaeEndEffectorConstants;
import frc.robot.subsystems.constants.ClimbConstants;
import frc.robot.subsystems.constants.CoralEndEffectorConstants;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffectors.AlgaeIntakeEndEffector;
import frc.robot.subsystems.endEffectors.AlgaeWristEndEffector;
import frc.robot.subsystems.endEffectors.CoralIntakeEndEffector;
import frc.robot.subsystems.endEffectors.CoralWristEndEffector;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Music;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AlgaeWristEndEffector algaeWristEndEffector;
  private final AlgaeIntakeEndEffector algaeIntakeEndEffector;
  private final CoralWristEndEffector coralWristEndEffector;
  private final CoralIntakeEndEffector coralIntakeEndEffector;
  // private final Climb climb;
  private final Elevator elevator;
  // private final LEDSubsystem led;

  private final Vision limelightLeft;
  private final Vision limelightRight;
  private final Vision limelightCenter;
  // private final Vision arducamOne;
  // private final Vision arducamTwo;
  
  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

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
        
              algaeWristEndEffector = AlgaeWristEndEffector.createInstance(AlgaeEndEffectorConstants.algaeWristMotor);

              algaeIntakeEndEffector = AlgaeIntakeEndEffector.createInstance(AlgaeEndEffectorConstants.algaeIntakeMotor, AlgaeEndEffectorConstants.canRange);

              coralWristEndEffector = CoralWristEndEffector.createInstance(CoralEndEffectorConstants.coralWristMotor);

              coralIntakeEndEffector = CoralIntakeEndEffector.createInstance(CoralEndEffectorConstants.coralIntakeMotor, CoralEndEffectorConstants.canRange);

              // climb = Climb.createInstance(ClimbConstants.climbMotor);

              elevator = Elevator.createInstance(
                ElevatorConstants.coralElevatorMotor, 
                ElevatorConstants.coralElevatorFollowerMotor, 
                ElevatorConstants.algaeElevatorMotor);
        
                limelightLeft = new Vision(VisionConstants.LIMELIGHT_LEFT);
                limelightRight = new Vision(VisionConstants.LIMELIGHT_RIGHT);
                limelightCenter = new Vision(VisionConstants.LIMELIGHT_CENTER);
                // arducamOne = new Vision(VisionConstants.ARDUCAM_ONE);
                // arducamTwo = new Vision(VisionConstants.ARDUCAM_TWO);
        
              // led = new LEDSubsystem();

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
    
                algaeWristEndEffector = AlgaeWristEndEffector.createInstance(new MotorIO() {});

                algaeIntakeEndEffector = AlgaeIntakeEndEffector.createInstance(new MotorIO() {}, new SensorIO() {});
  
                coralWristEndEffector = CoralWristEndEffector.createInstance(new MotorIO() {});
  
                coralIntakeEndEffector = CoralIntakeEndEffector.createInstance(new MotorIO() {}, new SensorIO() {});
  
                // climb = Climb.createInstance(new MotorIO() {});
  
                elevator = Elevator.createInstance(new MotorIO() {}, new MotorIO() {}, new MotorIO() {});
        
                    limelightLeft = new Vision(new VisionIO() {});
                    limelightRight = new Vision(new VisionIO() {});
                    limelightCenter = new Vision(new VisionIO() {});
                    // arducamOne = new Vision(new VisionIO() {});
                    // arducamTwo = new Vision(new VisionIO() {});
          
                // led = new LEDSubsystem();

                break;
            }

            configureDefaultBindings();
            configureDriverBindings();
            configureOperatorBindings();

            Dashboard.addWidgets(shuffleboardTab);
  }
        
  private void configureDefaultBindings() {
    drive.setDefaultCommand(
        DriveCommands.acceptTeleopFieldOriented(driver, true));
    
    coralWristEndEffector.setDefaultCommand(GlobalCommands.defaultCoralWristEndEffector());
    algaeWristEndEffector.setDefaultCommand(GlobalCommands.defaultAlgaeWristEndEffector());
    algaeIntakeEndEffector.setDefaultCommand(GlobalCommands.defaultAlgaeIntakeEndEffector());
    coralIntakeEndEffector.setDefaultCommand(GlobalCommands.defaultCoralIntakeEndEffector());
    // climb.setDefaultCommand(GlobalCommands.defaultClimb());

    elevator.setDefaultCommand(GlobalCommands.defaultElevator());
  }

  private void configureDriverBindings() {
    driver.x().whileTrue(GlobalCommands.algaeProcessorDrive());

    driver.rightTrigger(DriveConstants.TRIGGER_DEADBAND).whileTrue(
      DriveCommands.autoScoreToClosestReefPoseRight()
    );

    driver.a().whileTrue(Autons.centerOnePiecePlusAlgae());

    // Commands.deferredProxy(() -> 
    // Commands.sequence(
    //   AutonCommands.driveAndAutoScoreInAuton(FieldLayout.findClosestReefPoseRight()),
    //   AutonCommands.descoreAlgaeFromReefPosition(FieldLayout.findClosestReefPoseRight())
    // )
    // )

    driver.leftTrigger(DriveConstants.TRIGGER_DEADBAND).whileTrue(
        DriveCommands.autoScoreToClosestReefPoseLeft()
    );

    driver.start().onTrue(
        Commands.runOnce(() -> drive.setPose(new Pose2d()), drive).ignoringDisable(true));

    driver.leftBumper().onTrue(
      GlobalCommands.algaeRelease()
    );

    driver.rightBumper().onTrue(
      GlobalCommands.coralReleaseNoRequire()
    );

    driver.povRight().whileTrue(GlobalCommands.deploy());

    driver.povDown().whileTrue(GlobalCommands.fastRetract());

    driver.povLeft().whileTrue(GlobalCommands.slowReset());

    driver.y().whileTrue(GlobalCommands.algaeGroundIntake());
  }

  private void configureOperatorBindings() {
    operator.leftTrigger(DriveConstants.TRIGGER_DEADBAND).whileTrue(GlobalCommands.humanPlayerIntake());
    
    operator.rightTrigger(DriveConstants.TRIGGER_DEADBAND).whileTrue(GlobalCommands.lollipopPickup());

    operator.leftBumper().whileTrue(GlobalCommands.algaeBarge());

    operator.rightBumper().whileTrue(GlobalCommands.algaeProcessor());

    operator.y().whileTrue(GlobalCommands.coralL4NoAlgaeAutoScore()); // Algae L3 + Coral L4
    
    operator.b().whileTrue(GlobalCommands.coralL3NoAlgaeAutoScore()); // Coral L3

    operator.x().whileTrue(GlobalCommands.coralL2AutoScore()); // Coral L2

    operator.a().whileTrue(GlobalCommands.coralL3AutoScore()); // Algae L2

    operator.povUp().whileTrue(GlobalCommands.coralL4AutoScore()); // Coral L4
    
    operator.povRight().whileTrue(
      ElevatorCommands.resetElevatorCommand()
    );

    operator.povLeft().onTrue(
      ElevatorCommands.resetElevatorHighCommand()
    );

    operator.back().whileTrue(GlobalCommands.coralL2());

    operator.start().whileTrue(GlobalCommands.coralL3());

    operator.povDown().whileTrue(GlobalCommands.coralL4());

    operator.button(10).whileTrue(GlobalCommands.coralL1());

    operator.button(9).whileTrue(GlobalCommands.algaeGroundIntake());

    // operator.leftStick().or(operator.rightStick()).toggleOnTrue(GlobalCommands.coralL4NoAlgae());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Dashboard.getAutonomousCommand();
  }

         //Named commands
  // public void registerNamedCommands() {
  //   NamedCommands.registerCommand("Elevator + score L4", GlobalCommands.coralL4());
  //   NamedCommands.registerCommand("Coral HP intake", GlobalCommands.humanPlayerIntake());
  // }
}