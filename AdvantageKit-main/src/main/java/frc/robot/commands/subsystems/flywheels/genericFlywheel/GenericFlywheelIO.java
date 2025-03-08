package frc.robot.subsystems.flywheels.genericFlywheel;

import org.littletonrobotics.junction.AutoLog;

public interface GenericFlywheelIO {
  @AutoLog
  class GenericFlywheelIOInputs {
    public boolean motorConnected = false;
    public double voltage = 0.0;
    public double current = 0.0;
    public double position = 0.0;
    public double veloctiy = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(GenericFlywheelIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
