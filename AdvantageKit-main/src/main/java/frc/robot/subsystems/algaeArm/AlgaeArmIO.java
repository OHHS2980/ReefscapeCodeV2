package frc.robot.subsystems.algaeArm;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeArmIO {
  @AutoLog
  public static class AlgaeArmIOInputs {
    public double armPositionRad = 0;
    public double armVelocityRadPerSec = 0;
    public double absoluteEncoderConnected = 0;

    public double leftMotorAppliedVolts = 0;
    public double leftMotorCurrentAmps = 0;
    public double leftMotorTempC = 0;

    public double rightMotorAppliedVolts = 0;
    public double rightMotorCurrentAmps = 0;
    public double rightMotorTempC = 0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(AlgaeArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double voltage) {}

  /** Run to setpoint angle in radians */
  default void setSetpoint(double positionRads, double velocityRadPerSec) {}

  /** Set velocity PID constants. */
  public default void configurePID(
      double kP, double kI, double kD, double kV, double kA, double kG, double kS) {}
}
