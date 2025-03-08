package frc.robot.subsystems.superstructure.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ElevatorIONeo implements ElevatorIO {
  SparkMax leftMotor, rightMotor;
  RelativeEncoder leftEncoder, rightEncoder;
  PIDController controller = new PIDController(0, 0, 0);

  public ElevatorIONeo() {
    leftMotor = new SparkMax(41, MotorType.kBrushless);
    rightMotor = new SparkMax(42, MotorType.kBrushless);

    leftMotor.configure(
        new SparkMaxConfig().inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightMotor.configure(
        new SparkMaxConfig().follow(41),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected = true;
    inputs.leftVoltage = leftMotor.getAppliedOutput();
    inputs.leftCurrent = leftMotor.getOutputCurrent();
    inputs.leftPosition = leftEncoder.getPosition();
    inputs.leftVeloctiy = leftEncoder.getVelocity();
    inputs.leftTempC = leftMotor.getMotorTemperature();

    inputs.rightMotorConnected = true;
    inputs.rightVoltage = rightMotor.getAppliedOutput();
    inputs.rightCurrent = rightMotor.getOutputCurrent();
    inputs.rightPosition = rightEncoder.getPosition();
    inputs.rightVeloctiy = rightEncoder.getVelocity();
    inputs.rightTempC = rightMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
  }

  @Override
  public void setReference(State goalState) {
    leftMotor.setVoltage(
        controller.calculate(leftEncoder.getPosition(), (62.7035 / 0.803) * goalState.position));
  }

  @Override
  public void configurePID(
      double kP, double kI, double kD, double kV, double kA, double kG, double kS) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }
}
