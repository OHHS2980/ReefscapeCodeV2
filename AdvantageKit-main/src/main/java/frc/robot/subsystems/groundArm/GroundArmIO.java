package frc.robot.subsystems.groundArm;

import org.littletonrobotics.junction.AutoLog;

public interface GroundArmIO {
  @AutoLog
  public static class GroundArmIOInputs {
    public double armPositionRad = 0;
    public double armVelocityRadPerSec = 0;
    public double motorAppliedVolts = 0;
    public double motorCurrentAmps = 0;
    public double motorTempC = 0;
    public double absoluteEncoderConnected = 0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(GroundArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double voltage) {}

  /** Run to setpoint angle in radians */
  default void setSetpoint(double positionRads, double velocityRadPerSec) {}

  /** Set velocity PID constants. */
  public default void configurePID(
      double kP, double kI, double kD, double kV, double kA, double kG, double kS) {}
}
