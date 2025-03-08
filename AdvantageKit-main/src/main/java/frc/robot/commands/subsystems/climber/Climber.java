package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    Logger.recordOutput("Climber/name", getName());
    Logger.recordOutput("Climber/subsystem", getSubsystem());
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public Command extend() {
    return Commands.runOnce(() -> setVoltage(4), this);
  }

  public Command retract() {
    return Commands.runOnce(() -> setVoltage(-4), this);
  }

  public Command stop() {
    return Commands.runOnce(() -> setVoltage(0), this);
  }
}
