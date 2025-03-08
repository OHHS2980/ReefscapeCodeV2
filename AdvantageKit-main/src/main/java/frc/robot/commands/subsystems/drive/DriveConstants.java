package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public record PIDConfig(double P, double I, double D) {}

  public static final double maxSpeedMetersPerSec = 7.0;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(22.75);
  public static final double trackLength = Units.inchesToMeters(22.75);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, trackLength / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, trackLength / 2.0),
        new Translation2d(trackWidth / 2.0, -trackLength / 2.0),
        new Translation2d(-trackWidth / 2.0, trackLength / 2.0),
        new Translation2d(-trackWidth / 2.0, -trackLength / 2.0)
      };

  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  // Zeroed rotation values for each module, see setup instructions Negative of values in ascope
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-5.969);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.114);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(1.055);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-1.838);

  public static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.003));

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 11;
  public static final int frontRightDriveCanId = 13;
  public static final int backLeftDriveCanId = 15;
  public static final int backRightDriveCanId = 17;

  public static final int frontLeftTurnCanId = 10;
  public static final int frontRightTurnCanId = 12;
  public static final int backLeftTurnCanId = 14;
  public static final int backRightTurnCanId = 16;

  public static final int frontLeftTurnEncoderCanId = 21;
  public static final int frontRightTurnEncoderCanId = 22;
  public static final int backLeftTurnEncoderCanId = 23;
  public static final int backRightTurnEncoderCanId = 24;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double driveMotorReduction = 6.75;
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final PIDConfig drivePID = new PIDConfig(0.1, 0.0, 0.0);
  public static final PIDConfig driveSimPID = new PIDConfig(0.082416, 0.0, 0.0);
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.748;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.13968;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 150.0 / 7.0;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final PIDConfig turnPID = new PIDConfig(20.0, 0.0, 0.0);
  public static final PIDConfig turnSimPID = new PIDConfig(8.0, 0.0, 0.0);
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 45;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));
}
