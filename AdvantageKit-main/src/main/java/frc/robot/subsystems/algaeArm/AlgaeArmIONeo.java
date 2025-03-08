package frc.robot.subsystems.algaeArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeArmIONeo implements AlgaeArmIO {
  SparkMax leftArmMotor, rightArmMotor;
  RelativeEncoder leftArmEncoder, rightArmEncoder;

  DutyCycleEncoder encoder;

  ArmFeedforward ffModel = new ArmFeedforward(0, 0, 0, 0);
  PIDController pidController = new PIDController(0, 0, 0);
  Rotation2d offset = new Rotation2d();

  public AlgaeArmIONeo() {
    leftArmMotor = new SparkMax(51, MotorType.kBrushless);
    rightArmMotor = new SparkMax(52, MotorType.kBrushless);

    leftArmMotor.configure(
        new SparkMaxConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    rightArmMotor.configure(
        new SparkMaxConfig().follow(51, true),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder = new DutyCycleEncoder(1);

    leftArmEncoder = leftArmMotor.getEncoder();
    rightArmEncoder = rightArmMotor.getEncoder();
  }

  @Override
  public void updateInputs(AlgaeArmIOInputs inputs) {
    inputs.armPositionRad = getRadians();
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftArmEncoder.getVelocity());
  }

  @Override
  public void setVoltage(double voltage) {
    leftArmMotor.setVoltage(voltage);
  }

  @Override
  public void setSetpoint(double positionRads, double velocityRadPerSec) {
    leftArmMotor.setVoltage(
        pidController.calculate(getRadians(), positionRads)
            + ffModel.calculate(getRadians(), velocityRadPerSec));
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
