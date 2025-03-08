package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
  @Getter private final Elevator elevator;

  @RequiredArgsConstructor
  public enum SuperstructureGoals {
    STOW(0.0),
    POS1(0.2);
    private final double elevatorSetpoint;

    private double getMeters() {
      return elevatorSetpoint;
    }
  }

  @Getter
  @Setter
  @AutoLogOutput(key = "Superstructure/Goal")
  private SuperstructureGoals superstructureGoal = SuperstructureGoals.STOW;

  public Superstructure(ElevatorIO elevatorIO) {
    elevator = new Elevator(elevatorIO);
  }

  @Override
  public void periodic() {
    elevator.setReference(superstructureGoal.getMeters(), 0);
  }
}
