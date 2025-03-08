package frc.robot.subsystems.groundArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;

public class GroundArmIONeo implements GroundArmIO {
  SparkMax armMotor;
  RelativeEncoder armEncoder;
  SparkClosedLoopController armController;

  DutyCycleEncoder encoder;

  ArmFeedforward ffModel = new ArmFeedforward(0, 0, 0, 0);
  PIDController pidController = new PIDController(0, 0, 0);
  Rotation2d offset = Rotation2d.fromRadians(3.26625);

  Rotation2d motorOffset;

  public GroundArmIONeo() {
    armMotor = new SparkMax(32, MotorType.kBrushless);

    armMotor.configure(
        new SparkMaxConfig().inverted(true),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    armController = armMotor.getClosedLoopController();
    armEncoder = armMotor.getEncoder();

    encoder = new DutyCycleEncoder(0);
    encoder.setInverted(false);

    motorOffset = Rotation2d.fromRadians(getRadians());
  }

  @Override
  public void updateInputs(GroundArmIOInputs inputs) {
    inputs.armPositionRad = getRadians();
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity());

    inputs.motorAppliedVolts = armMotor.getAppliedOutput();
    inputs.motorCurrentAmps = armMotor.getOutputCurrent();

    inputs.motorTempC = armMotor.getMotorTemperature();

    Logger.recordOutput("Ground Motor Pos", armEncoder.getPosition());
  }

  @Override
  public void setVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  @Override
  public void setSetpoint(double positionRads, double velocityRadPerSec) {
    armMotor.setVoltage(pidController.calculate(getRadians(), positionRads));
    // (armEncoder.getPosition() * 0.276121337827) - motorOffset.getRadians()
    // + ffModel.calculate(getRadians(), velocityRadPerSec));
  }

  @Override
  public void configurePID(
      double kP, double kI, double kD, double kV, double kA, double kG, double kS) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    ffModel = new ArmFeedforward(kS, kG, kV, kA);
  }

  private double getRadians() {
    return Units.rotationsToRadians(encoder.get()) - offset.getRadians();
  }
}
