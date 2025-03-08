package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheels.genericFlywheel.GenericFlywheelIO;
import frc.robot.subsystems.flywheels.genericFlywheel.GenericFlywheelIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CoralFunnel extends SubsystemBase {
  private final GenericFlywheelIO io;
  private final GenericFlywheelIOInputsAutoLogged inputs = new GenericFlywheelIOInputsAutoLogged();

  /** Creates a new Coral Funnel. */
  public CoralFunnel(GenericFlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Coral Funnel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public Command intake() {
    return Commands.runOnce(() -> setVoltage(-6), this);
  }

  public Command outtake() {
    return Commands.runOnce(() -> setVoltage(12), this);
  }

  public Command stop() {
    return Commands.runOnce(() -> setVoltage(0), this);
  }
}
