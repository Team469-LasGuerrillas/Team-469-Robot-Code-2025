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

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.hal.SimDevice.Direction;
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
import frc.robot.autons.Autons;
import frc.robot.commandfactories.AlgaeEndEffectorCommands;
import frc.robot.commandfactories.AutonCommands;
import frc.robot.commandfactories.CoralEndEffectorCommands;
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
import frc.robot.subsystems.constants.SensorConstants;
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
  private final AlgaeWristEndEffector algaeWristEndEffector;
  private final AlgaeIntakeEndEffector algaeIntakeEndEffector;
  private final CoralWristEndEffector coralWristEndEffector;
  private final CoralIntakeEndEffector coralIntakeEndEffector;
  private final Climb climb;
  private final Elevator elevator;

  private final Vision limelightLeft;
  private final Vision limelightRight;
  private final Vision arducamOne;
  private final Vision arducamTwo;

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

              climb = Climb.createInstance(ClimbConstants.climbMotor);

              elevator = Elevator.createInstance(
                ElevatorConstants.coralElevatorMotor, 
                ElevatorConstants.coralElevatorFollowerMotor, 
                ElevatorConstants.algaeElevatorMotor);
        
                limelightLeft = new Vision(VisionConstants.LIMELIGHT_LEFT);
                limelightRight = new Vision(VisionConstants.LIMELIGHT_RIGHT);
                arducamOne = new Vision(VisionConstants.ARDUCAM_ONE);
                arducamTwo = new Vision(VisionConstants.ARDUCAM_TWO);
        
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
  
                climb = Climb.createInstance(new MotorIO() {});
  
                elevator = Elevator.createInstance(new MotorIO() {}, new MotorIO() {}, new MotorIO() {});
        
                    limelightLeft = new Vision(new VisionIO() {});
                    limelightRight = new Vision(new VisionIO() {});
                    arducamOne = new Vision(new VisionIO() {});
                    arducamTwo = new Vision(new VisionIO() {});
          
                break;
            }

    Dashboard.addWidgets(shuffleboardTab);
        
      configureDefaultBindings();
      configureDriverBindings();
      configureOperatorBindings();

      // registerNamedCommands();
  }
        
  private void configureDefaultBindings() {
    drive.setDefaultCommand(
        DriveCommands.acceptTeleopFieldOriented(driver, true));
    
    coralWristEndEffector.setDefaultCommand(GlobalCommands.defaultCoralWristEndEffector());
    algaeWristEndEffector.setDefaultCommand(GlobalCommands.defaultAlgaeWristEndEffector());
    algaeIntakeEndEffector.setDefaultCommand(GlobalCommands.defaultAlgaeIntakeEndEffector());
    coralIntakeEndEffector.setDefaultCommand(GlobalCommands.defaultCoralIntakeEndEffector());
    climb.setDefaultCommand(GlobalCommands.defaultClimb());

    elevator.setDefaultCommand(GlobalCommands.defaultElevator());
  }

  private void configureDriverBindings() {
    driver.a().whileTrue(Autons.startEFF());

    driver.rightTrigger(DriveConstants.TRIGGER_DEADBAND).whileTrue(
        DriveCommands.pidToClosestReefPoseRight()
    );

    driver.leftTrigger(DriveConstants.TRIGGER_DEADBAND).whileTrue(
        DriveCommands.pidToClosestReefPoseLeft()
    );

    driver.start().onTrue(
        Commands.runOnce(() -> drive.setPose(new Pose2d()), drive).ignoringDisable(true));

    driver.leftBumper().whileTrue(
      GlobalCommands.algaeRelease()
    );

    driver.rightBumper().onTrue(
      GlobalCommands.coralRelease()
    );
  }

  private void configureOperatorBindings() {
    operator.leftTrigger(DriveConstants.TRIGGER_DEADBAND).whileTrue(GlobalCommands.humanPlayerIntake());
    
    operator.rightTrigger(DriveConstants.TRIGGER_DEADBAND).whileTrue(GlobalCommands.algaeGroundIntake());

    operator.leftBumper().whileTrue(GlobalCommands.algaeBarge());

    operator.rightBumper().whileTrue(GlobalCommands.algaeProcessor());

    operator.y().whileTrue(GlobalCommands.coralL4());
    
    operator.b().whileTrue(GlobalCommands.coralL3());

    operator.x().whileTrue(GlobalCommands.coralL2());

    operator.a().whileTrue(GlobalCommands.coralL1());

    operator.povUp().whileTrue(GlobalCommands.deploy());

    operator.povDown().whileTrue(GlobalCommands.fastRetract());

    operator.povLeft().or(operator.povRight()).whileTrue(GlobalCommands.slowRetract());
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