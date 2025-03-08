// Copyright 2021-2025 FRC 6328
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
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKracken;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOComp;
import frc.robot.subsystems.drive.module.ModuleIOMaple;
import frc.robot.subsystems.flywheels.AlgaeSubsystem;
import frc.robot.subsystems.flywheels.CoralFunnel;
import frc.robot.subsystems.flywheels.genericFlywheel.GenericFlywheelIO;
import frc.robot.subsystems.flywheels.genericFlywheel.GenericFlywheelIOKracken;
import frc.robot.subsystems.flywheels.genericFlywheel.GenericFlywheelIONeo;
import frc.robot.subsystems.flywheels.genericFlywheel.GenericFlywheelIOSim;
import frc.robot.subsystems.groundArm.GroundArm;
import frc.robot.subsystems.groundArm.GroundArm.ArmGoal;
import frc.robot.subsystems.groundArm.GroundArmIONeo;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.io.IOException;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  Drive drive;
  Vision vision;
  AlgaeSubsystem algae;
  GroundArm groundIntakeArm;
  Climber climber;
  CoralFunnel funnel;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setupSubsystems();

    NamedCommands.registerCommand(
        "Outtake Coral",
        Commands.sequence(
            Commands.parallel(funnel.outtake(), Commands.waitSeconds(0.7)), funnel.stop()));

    NamedCommands.registerCommand(
        "Intake Coral", Commands.parallel(funnel.intake(), Commands.waitSeconds(0.7)));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

    PathPlannerAuto testingAuto = new PathPlannerAuto("Testing Auto");
    PathPlannerAuto center = new PathPlannerAuto("Center");
    PathPlannerAuto left1 = new PathPlannerAuto("Left1");
    PathPlannerAuto right1 = new PathPlannerAuto("Left1", true);
    PathPlannerAuto left2 = new PathPlannerAuto("Left2");
    PathPlannerAuto right2 = new PathPlannerAuto("Left2", true);
    PathPlannerAuto left3 = new PathPlannerAuto("Left3");
    PathPlannerAuto right3 = new PathPlannerAuto("Left3", true);
    PathPlannerAuto left4 = new PathPlannerAuto("Left4");
    PathPlannerAuto right4 = new PathPlannerAuto("Left4", true);

    autoChooser.addOption("Testing Auto", testingAuto);
    autoChooser.addOption("Center", center);
    autoChooser.addOption("Left 1", left1);
    autoChooser.addOption("Right 1", right1);
    autoChooser.addOption("Left 2", left2);
    autoChooser.addOption("Right 2", right2);
    autoChooser.addOption("Left 3", left3);
    autoChooser.addOption("Right 3", right3);
    autoChooser.addOption("Left 4", left4);
    autoChooser.addOption("Right 4", right4);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void setupSubsystems() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT:
          drive =
              new Drive(
                  new GyroIONavX(),
                  new ModuleIOComp(0),
                  new ModuleIOComp(1),
                  new ModuleIOComp(2),
                  new ModuleIOComp(3));
          funnel = new CoralFunnel(new GenericFlywheelIONeo(37, new SparkMaxConfig()));
          climber = new Climber(new ClimberIOKracken());
          groundIntakeArm = new GroundArm(new GroundArmIONeo());
          algae = new AlgaeSubsystem(new GenericFlywheelIOKracken(31, "canivore"));
          break;

        case SIMBOT:
          climber = new Climber(new ClimberIOSim());
          drive =
              new Drive(
                  new GyroIOSim(SimulationVisualizer.getInstance().getGyroSimulation()),
                  new ModuleIOMaple(
                      SimulationVisualizer.getInstance().getSwerveModuleSimulation(0)),
                  new ModuleIOMaple(
                      SimulationVisualizer.getInstance().getSwerveModuleSimulation(1)),
                  new ModuleIOMaple(
                      SimulationVisualizer.getInstance().getSwerveModuleSimulation(2)),
                  new ModuleIOMaple(
                      SimulationVisualizer.getInstance().getSwerveModuleSimulation(3)));
          vision =
              new Vision(
                  new VisionIOPhotonVisionSim(
                      VisionConstants.camera0Name, VisionConstants.robotToCamera1));
          algae = new AlgaeSubsystem(new GenericFlywheelIOSim());
          funnel = new CoralFunnel(new GenericFlywheelIOSim());
          break;

        default:
          break;
      }
    }

    // No-op implementations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    if (vision == null) {
      vision = new Vision(new VisionIO() {});
    }

    if (algae == null) {
      algae = new AlgaeSubsystem(new GenericFlywheelIO() {});
    }

    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }

    if (funnel == null) {
      funnel = new CoralFunnel(new GenericFlywheelIO() {});
    }
  }

  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Reset gyro to 0 when B button is pressed
    controller
        .triangle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .setPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose2d().getTranslation(),
                                    DriverStation.getAlliance().get() == Alliance.Red
                                        ? Rotation2d.fromDegrees(0)
                                        : Rotation2d.fromDegrees(180))))
                .ignoringDisable(true));

    controller.L2().onTrue(funnel.intake()).onFalse(funnel.stop());
    controller.R2().onTrue(funnel.outtake()).onFalse(funnel.stop());

    controller
        .L1()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(() -> groundIntakeArm.setGoal(ArmGoal.INTAKE)), algae.intake()))
        .onFalse(
            Commands.parallel(
                Commands.runOnce(() -> groundIntakeArm.setGoal(ArmGoal.STOW)), algae.stop()));

    controller.R1().onTrue(algae.outtake()).onFalse(algae.stop());

    controller.povUp().onTrue(climber.extend()).onFalse(climber.stop());
    controller.povDown().onTrue(climber.retract()).onFalse(climber.stop());
  }

  public Command getAutoCommand() {
    // return DriveCommands.joystickDrive(drive, () -> 0.2, () -> 0, () -> 0)
    // .deadlineWith(Commands.waitSeconds(3));
    return autoChooser.get();
  }

  private Pose2d[] getPathPose2ds() {
    try {
      List<Pose2d> list = PathPlannerPath.fromPathFile("Example Path").getPathPoses();
      Pose2d[] result = new Pose2d[list.size()];
      for (int i = 0; i < result.length; i++) {
        result[i] = list.get(i);
      }
      return result;
    } catch (FileVersionException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    } catch (ParseException e) {
      e.printStackTrace();
    } finally {

    }
    return null;
  }

  private void addTuningAutos() {
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    SimulationVisualizer.getInstance()
                        .setPositiion(new Pose2d(0, -1.5, new Rotation2d()))),
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    SimulationVisualizer.getInstance()
                        .setPositiion(new Pose2d(0, -1.5, new Rotation2d()))),
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    SimulationVisualizer.getInstance()
                        .setPositiion(new Pose2d(0, -1.5, new Rotation2d()))),
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    SimulationVisualizer.getInstance()
                        .setPositiion(new Pose2d(0, -1.5, new Rotation2d()))),
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)));

    PathPlannerAuto testingAuto = new PathPlannerAuto("New Auto");
    autoChooser.addOption("Pathplanner Test", testingAuto);
  }
}
