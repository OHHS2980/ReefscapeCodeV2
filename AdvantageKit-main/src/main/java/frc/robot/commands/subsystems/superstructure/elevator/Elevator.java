package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final TrapezoidProfile profile = new TrapezoidProfile(new Constraints(5, 5));
  private boolean autoMove = false;
  private State goalState = new State(0, 0);

  public Elevator(ElevatorIO io) {
    this.io = io;
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        io.configurePID(0.01, 0, 0, 0, 0, 0, 0);
        break;
      case SIM:
        io.configurePID(200, 0, 0, 4, 0, 0.3, 0);
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/name", getName());
    Logger.recordOutput("Elevator/subsystem", getSubsystem());

    if (autoMove) {
      io.setReference(
          profile.calculate(
              Constants.loopPeriodSecs,
              new State(inputs.leftPosition, inputs.leftVeloctiy),
              goalState));
      // Replace inputs.leftPosition, inputs.leftVeloctiy with io.getPositionMeters() and
      // io.getVelocityMetersPerSecond
    }
  }

  public void setVoltage(double voltage, boolean safe) {
    autoMove = false;
    if (safe) {
      io.setVoltage(0);
    } else {
      io.setVoltage(voltage);
    }
  }

  public void stop() {
    io.stop();
  }

  public void setReference(double height, double velocity) {
    autoMove = true;
    goalState = new State(height, velocity);
  }
}
