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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Dashboard;
import frc.lib.drivecontrollers.FieldRobotSpeedsConversion;
import frc.lib.drivecontrollers.HeadingController;
import frc.lib.drivecontrollers.LinearController;
import frc.lib.drivecontrollers.TeleopDriveController;
import frc.lib.util.Clock;
import frc.lib.util.FieldLayout;
import frc.lib.util.MonkeyState;
import frc.lib.util.Station;
import frc.lib.util.FieldLayout.ReefPositions;
import frc.lib.util.hardware.QuestNavUtil;
import frc.lib.util.math.InterpolatorUtil;
import frc.lib.util.math.ToleranceUtil;
import frc.lib.util.math.estimator.SequencingSwerveDrivePoseEstimator;
import frc.lib.util.math.odometry.OdometryType;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.util.LocalADStarAK;

import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static Drive instance;

  public enum DriveMode {
    /** Driving with input from driver joysticks. */
    TELEOP,

    /** Driving with input from driver joysticks but there is a eading controller. */
    HEADING,

    /** Driving based on a pathplanner provided speeds. */
    PATHPLANNER,

    /** Driving by combining input from driver joysticks and pathplanner */
    AIMASSIST,

    PID_TO_POINT,

    AUTON
  }

  public enum CoastRequest {
    BRAKE,
    COAST
  }

  Pose2d lastGoodKnownPose = new Pose2d();
  int poseResetCounter = 0;

  class DriveState implements MonkeyState {
    public Pose3d robotPose;
    public DriveMode driveMode;

    private TimeInterpolatableBuffer<Pose3d> poseBuffer = TimeInterpolatableBuffer.createBuffer(2);
    private TimeInterpolatableBuffer<DriveMode> modeBuffer =
        TimeInterpolatableBuffer.createBuffer(new DriveModeInterpolator(), 2);

    @Override
    public void addState(double timestamp) {
      poseBuffer.addSample(timestamp, robotPose);
      modeBuffer.addSample(timestamp, currentDriveMode);
    }

    @Override
    public MonkeyState getState(double timestamp) {
      DriveState legacyState = new DriveState();
      legacyState.robotPose = poseBuffer.getSample(timestamp).get();
      legacyState.driveMode = modeBuffer.getSample(timestamp).get();

      return legacyState;
    }
  }

  private class DriveModeInterpolator implements Interpolator<DriveMode> {
    @Override
    public DriveMode interpolate(DriveMode startValue, DriveMode endValue, double t) {
      if (t > 0.5) {
        return endValue;
      } else {
        return startValue;
      }
    }
  }

  public DriveState state = new DriveState();

  // The robot's current drivemode
  private DriveMode currentDriveMode = DriveMode.TELEOP;
  private CoastRequest coastRequest = CoastRequest.BRAKE;

  // Controllers for driving
  private ChassisSpeeds desiredSpeeds;
  private ChassisSpeeds autoSpeeds;

  private final TeleopDriveController teleopDriveController;
  private final PPHolonomicDriveController ppHolonomicDriveController;
  private HeadingController headingController = null;
  private LinearController linearController = null;

  private double aimAssistWeight;

  private PathfindingCommand pathfindingCommand;

  public int isOnTargetLoopCount = 0;

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

  // PathPlanner config constants
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          DriveConstants.ROBOT_MASS_KG,
          DriveConstants.ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              DriveConstants.TELEOP_MAX_LINEAR_VELOCITY,
              DriveConstants.WHEEL_COF,
              DCMotor.getKrakenX60(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  // Pathplanner pathfinding constants
  private static final PathConstraints CONSTRAINTS =
      new PathConstraints(
          DriveConstants.TELEOP_MAX_LINEAR_VELOCITY,
          DriveConstants.MAX_LINEAR_ACCEL_FINE,
          DriveConstants.TELEOP_MAX_ANGULAR_VELOCITY,
          DriveConstants.MAX_ANGULAR_ACCEL);

  // Phonix odometry thread lockout
  static final Lock odometryLock = new ReentrantLock();

  // Gyro
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Swerve
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint currentSetpoint;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private SequencingSwerveDrivePoseEstimator poseEstimator =
      new SequencingSwerveDrivePoseEstimator(
          kinematics, rawGyroRotation, lastModulePositions, new Pose3d(-0.254, 0, 0, new Rotation3d(Math.PI, 0 , 0)), OdometryType.VR_ODOMETRY);

  public static void createInstance(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    instance = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
  }

  public static Drive getInstance() {
    return instance;
  }


  private PPHolonomicDriveController pppppp;

  private Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Swerve setpoint generator
    setpointGenerator = new SwerveSetpointGenerator(PP_CONFIG, getMaxAngularSpeedRadPerSec());
    currentSetpoint =
        new SwerveSetpoint(
            getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(PP_CONFIG.numModules));

    // Pathplanner holonmic driver config
    ppHolonomicDriveController =
        new PPHolonomicDriveController(
            new PIDConstants(
                DriveConstants.PP_TRANSLATION_P,
                DriveConstants.PP_TRANSLATION_I,
                DriveConstants.PP_TRANSLATION_D),
            new PIDConstants(
                DriveConstants.PP_HEADING_P,
                DriveConstants.PP_HEADING_I,
                DriveConstants.PP_HEADING_D));

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        (pose) -> {},
        this::getChassisSpeeds,
        this::setAutoSpeedsInAuton,
        ppHolonomicDriveController,
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Configure controllers
    teleopDriveController = new TeleopDriveController();

    if(DriverStation.getAlliance().get() == Alliance.Blue) {
      setPose(new Pose2d(8, 6, Rotation2d.fromDegrees(0 + 120)));
    } else {
      setPose(new Pose2d(8, 2, Rotation2d.fromDegrees(180 + 120)));
    }
  }

  @Override
  public void periodic() {
    Dashboard.m_field.setRobotPose(getPose());
    // System.out.println("Goal: 4.95, 5.15" + "Actual: " + getPose().getTranslation());

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Update gyro alert
      gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

      // Apply update
      poseEstimator.updateWithTime(
          sampleTimestamps[i],
          rawGyroRotation,
          modulePositions,
          0,
          QuestNavUtil.getInstance().connected());

      switch (coastRequest) {
        case BRAKE -> {
          setBrakeMode(true);
          break;
        }
        case COAST -> {
          setBrakeMode(false);
          break;
        }
      }

      ChassisSpeeds teleopSpeeds = teleopDriveController.update();
      Logger.recordOutput("Drive/TeleopSpeeds", teleopSpeeds);

      desiredSpeeds = teleopSpeeds;
      switch (currentDriveMode) {
        case TELEOP -> {
          desiredSpeeds = teleopSpeeds;
          break;
        }

        case HEADING -> {
          desiredSpeeds = teleopSpeeds;
          desiredSpeeds.omegaRadiansPerSecond = headingController.update();
          break;
        }

        case PID_TO_POINT -> {
          desiredSpeeds = FieldRobotSpeedsConversion.fieldToRobotSpeeds(linearController.update(), getRotation());
          desiredSpeeds.omegaRadiansPerSecond = headingController.update();

          Logger.recordOutput("linearController/linearControllerDriveSubsystemVelocity", Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond));
          break;
        }

        case PATHPLANNER -> {
          if (autoSpeeds == null) {
            desiredSpeeds = teleopSpeeds;
          } else {
            desiredSpeeds = autoSpeeds;
          }
          break;
        }
          
        case AUTON -> {
          if (autoSpeeds == null) {
            desiredSpeeds = new ChassisSpeeds();
          } else {
            desiredSpeeds = autoSpeeds;
          }
          break;
        }

        case AIMASSIST -> {
          ChassisSpeeds aimAssistSpeeds = FieldRobotSpeedsConversion.fieldToRobotSpeeds(linearController.update(), getRotation());
          aimAssistSpeeds.omegaRadiansPerSecond = headingController.update();
          desiredSpeeds = InterpolatorUtil.chassisSpeeds(teleopSpeeds, aimAssistSpeeds, aimAssistWeight);
          break;
        }
      }
    }

    // Run the modules
    if (!DriverStation.isDisabled()) {
      runVelocity(desiredSpeeds);
    }

    Logger.recordOutput(
        "Drive/SwerveStates/Desired(b4 Poofs)", kinematics.toSwerveModuleStates(desiredSpeeds));
    Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
    Logger.recordOutput("Drive/DriveMode", currentDriveMode);

    Logger.recordOutput("Drive/AutoSpeeds", autoSpeeds);

    state.robotPose = new Pose3d(getPose());
    state.driveMode = currentDriveMode;
    state.addState(Clock.time());

    Pose2d currentPose = getPose();
    if (currentPose.getX() != currentPose.getX() 
        || currentPose.getY() != currentPose.getY()
        || currentPose.getRotation().getRadians() != currentPose.getRotation().getRadians()) {
        setPose(lastGoodKnownPose);
        poseResetCounter++;
    }
    else {
      lastGoodKnownPose = currentPose;
    }
  }

  @AutoLogOutput
  public double getPoseResetCounter() { return poseResetCounter; }
  
  private Command followPathInternal(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        this::getPose,
        this::getChassisSpeeds,
        this::setAutoSpeeds,
        ppHolonomicDriveController,
        PP_CONFIG,
        () -> Station.isRed(),
        this);
  }

  private Command findPathInternal(PathPlannerPath path) {
    return new PathfindThenFollowPath(
        path,
        CONSTRAINTS,
        this::getPose,
        this::getChassisSpeeds,
        this::setAutoSpeeds,
        ppHolonomicDriveController,
        PP_CONFIG,
        () -> Station.isRed(),
        this);
  }

  private void findPathInternal(Pose2d pose) {
    pathfindingCommand = new PathfindingCommand(
        pose,
        CONSTRAINTS,
        0.0,
        this::getPose,
        this::getChassisSpeeds,
        this::setAutoSpeeds,
        ppHolonomicDriveController,
        PP_CONFIG,
        this);
    
    pathfindingCommand.schedule();
  }

  private void setAutoSpeeds(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    autoSpeeds = speeds;    
  }

  private void setAutoSpeedsInAuton(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    autoSpeeds = speeds;
    currentDriveMode = DriveMode.AUTON;
  }

  public void setTeleopMode() {
    currentDriveMode = DriveMode.TELEOP;
  }

  public void setHeadingGoal(Supplier<Rotation2d> heading) {
    headingController = new HeadingController(heading);
    currentDriveMode = DriveMode.HEADING;
  }

  public void setPIDToPointGoal(Supplier<Pose2d> pose) {
    setPIDToPointGoal(pose, false);
  }

  public void setPIDToPointGoal(Supplier<Pose2d> pose, boolean runFineStep) {
    linearController = new LinearController(pose, runFineStep);
    headingController = new HeadingController(() -> pose.get().getRotation());
    currentDriveMode = DriveMode.PID_TO_POINT;
  }

  public void setPathfinding(Pose2d pose) {
    findPathInternal(pose);
    currentDriveMode = DriveMode.PATHPLANNER;
  }

  public void setAimAssist(Pose2d pose, double aimAssistWeight) {
    this.aimAssistWeight = aimAssistWeight;
    linearController = new LinearController(() -> pose, false);
    headingController = new HeadingController(() -> pose.getRotation());
    currentDriveMode = DriveMode.AIMASSIST;
  }

  public void clearMode() {
    if (currentDriveMode == DriveMode.PATHPLANNER) {
      pathfindingCommand.cancel();
    }

    headingController = null;
    linearController = null;

    currentDriveMode = DriveMode.TELEOP;
  }
  
  /**
   * Takes inputs from a joystick to drive the robot
   *
   * @param relativeX The X axis input
   * @param relativeY The Y axis input
   * @param omega The anglular axis input, cc positive
   * @param robotRelative Is it robot relative?
   */
  public void acceptTeleopInput(
      double relativeX, double relativeY, double omega, boolean robotRelative) {
    if (DriverStation.isTeleopEnabled()) {
      teleopDriveController.acceptDriveInput(relativeX, relativeY, omega, robotRelative);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public SwerveModuleState[] runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    currentSetpoint = setpointGenerator.generateSetpoint(currentSetpoint, speeds, 0.02);
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    SwerveModuleState[] optimizedSetpointTorques = new SwerveModuleState[4];

    // ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    // SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);


    for (int i = 0; i < 4; i++) {
      optimizedSetpointStates[i] = currentSetpoint.moduleStates()[i];
      optimizedSetpointStates[i].optimize(modules[i].getAngle());

      optimizedSetpointTorques[i] = new SwerveModuleState(0.0, optimizedSetpointStates[i].angle);
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        optimizedSetpointStates, DriveConstants.TELEOP_MAX_LINEAR_VELOCITY);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/SwerveStates/Setpoints", optimizedSetpointStates);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(optimizedSetpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    
    return optimizedSetpointStates;
  }

  public void applyFeedForwards(DriveFeedforwards feedforwards) {}

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  private void setBrakeMode(boolean enabled) {
    Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Velocity")
  private double getNetVelocity() {
    return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Field")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  /** Field relative velocity */
  // public Twist2d fieldVelocity() {
  //   return getFieldVelocity().toTwist2d(1);
  // }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/RobotPose")
  public Pose2d getPose() {
    return poseEstimator.getCurrentPoseEstimate();
  }

  @AutoLogOutput(key = "Odometry/TargetPose")
  public Pose2d getTargetPose() {
    if (linearController == null || headingController == null) return new Pose2d();

    return linearController.getTargetPose();
  }

  public double getLinearControllerError() {
    if (linearController == null) return 0;

    return linearController.getError();
  }

  @AutoLogOutput
  public double getErrorFromLinearControllerTargetMeters() {
    double robotDistanceFromReefCenter;
    if (Station.isRed()) 
      robotDistanceFromReefCenter = FieldLayout.REEF_CENTER_RED
        .getTranslation().getDistance(getPose().getTranslation());
    else robotDistanceFromReefCenter = FieldLayout.REEF_CENTER_BLUE
      .getTranslation().getDistance(getPose().getTranslation());
    
    double targetDistanceFromReefCenter;
    if (linearController == null) targetDistanceFromReefCenter = robotDistanceFromReefCenter;
    else targetDistanceFromReefCenter = linearController.getTargetDistanceFromReefCenter();

    return robotDistanceFromReefCenter - targetDistanceFromReefCenter;
  }

  public boolean isOnTarget() {
    if (linearController == null || headingController == null) return false;

    return
      linearController.atXGoal()
      && linearController.atYGoal()
      && headingController.atGoal();

  }

  public boolean isOnTarget(double linearTolerance, double headingToleranceDegrees) {
    if (linearController == null || headingController == null) return false;

    return
      linearController.getTargetPose().getTranslation().getDistance(getPose().getTranslation()) < linearTolerance
      && headingController.atGoal(headingToleranceDegrees);
  }

  public boolean isOnTarget(Pose2d targetPose, double linearTolerance, double headingToleranceDegrees) {
    return targetPose.getTranslation().getDistance(getPose().getTranslation()) < linearTolerance
      && ToleranceUtil.epsilonEqualsRadialDegrees(targetPose.getRotation().getDegrees(), getPose().getRotation().getDegrees(), headingToleranceDegrees);
  }

  public boolean isOnTarget(ReefPositions targetReefPosition, double linearTolerance, double headingToleranceDegrees) {
    Pose2d targetPose;

    if (FieldLayout.reefPositionPoseLeft.containsKey(targetReefPosition))
     targetPose = FieldLayout.reefPositionPoseLeft.get(targetReefPosition);
    else targetPose = FieldLayout.reefPositionPoseRight.get(targetReefPosition);
    
    return isOnTarget(targetPose, linearTolerance, headingToleranceDegrees);
  }

  public boolean isOnTarget(ReefPositions targetReefPosition, double linearTolerance, double headingToleranceDegrees, int numOfOnTargetLoops) {
    if (isOnTarget(targetReefPosition, linearTolerance, headingToleranceDegrees))
     isOnTargetLoopCount++;
    else isOnTargetLoopCount = 0;

    System.out.println("Drive on target: " + isOnTargetLoopCount);

    if (isOnTargetLoopCount >= numOfOnTargetLoops) return true;
    return false;
  }

  public Optional<Pose2d> getTimestampedPose(double timestamp) {
    return poseEstimator.getPoseEstimate(timestamp);
  }

  public Pose2d getRaw() {
    return poseEstimator.getRawPose();
  }
  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }
  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);

    if (Math.abs(getPose().getRotation().getDegrees() - pose.getRotation().getDegrees()) > 1) {
      setPose(pose);
    }
  }

  /** Returns if the vr is connected */
  @AutoLogOutput (key = "Odometry/VrConnected")
  public boolean getSecondaryConnected() {
    return poseEstimator.isSecondaryConnected();
  }


  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.TELEOP_MAX_LINEAR_VELOCITY;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DriveConstants.DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
