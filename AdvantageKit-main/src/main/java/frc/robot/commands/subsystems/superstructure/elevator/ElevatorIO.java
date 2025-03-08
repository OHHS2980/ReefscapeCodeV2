package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leftMotorConnected = false;
    public double leftVoltage = 0.0;
    public double leftCurrent = 0.0;
    public double leftPosition = 0.0;
    public double leftVeloctiy = 0.0;
    public double leftTempC = 0.0;

    public boolean rightMotorConnected = false;
    public double rightVoltage = 0.0;
    public double rightCurrent = 0.0;
    public double rightPosition = 0.0;
    public double rightVeloctiy = 0.0;
    public double rightTempC = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setReference(State goalState) {}

  public default void stop() {}

  public default void configurePID(
      double kP, double kI, double kD, double kV, double kA, double kG, double kS) {}
}
