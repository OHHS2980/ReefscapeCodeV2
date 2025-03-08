package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {
    public boolean motorConnected = false;
    public double voltage = 0.0;
    public double current = 0.0;
    public double position = 0.0;
    public double veloctiy = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
