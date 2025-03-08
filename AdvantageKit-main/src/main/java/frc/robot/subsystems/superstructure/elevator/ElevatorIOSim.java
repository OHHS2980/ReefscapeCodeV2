package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          10,
          10,
          Units.inchesToMeters(0.5),
          0,
          Units.inchesToMeters(30),
          true,
          0);
  private PIDController pid = new PIDController(0, 0, 0);
  private ElevatorFeedforward ffModel = new ElevatorFeedforward(0, 0, 0);
  private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(5, 5));

  boolean closedLoop = false;
  double appliedVoltage = 0.0;
  State goalState = new State(0, 0);

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      appliedVoltage =
          pid.calculate(sim.getPositionMeters(), goalState.position)
              + ffModel.calculate(goalState.velocity);
    }
    MathUtil.clamp(appliedVoltage, -12, 12);
    sim.setInputVoltage(appliedVoltage);
    sim.update(Constants.loopPeriodSecs);

    inputs.leftMotorConnected = true;
    inputs.leftVoltage = appliedVoltage;
    inputs.leftCurrent = sim.getCurrentDrawAmps();
    inputs.leftPosition = sim.getPositionMeters();
    inputs.leftVeloctiy = sim.getVelocityMetersPerSecond();

    inputs.rightMotorConnected = true;
    inputs.rightVoltage = appliedVoltage;
    inputs.rightCurrent = sim.getCurrentDrawAmps();
    inputs.rightPosition = sim.getPositionMeters();
    inputs.rightVeloctiy = sim.getVelocityMetersPerSecond();

    Logger.recordOutput(
        "Elevator/Elevator Base", new Pose3d(0, 0, sim.getPositionMeters(), new Rotation3d()));
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    appliedVoltage = voltage;
  }

  @Override
  public void setReference(State goalState) {
    closedLoop = true;
    this.goalState = goalState;
  }

  @Override
  public void configurePID(
      double kP, double kI, double kD, double kV, double kA, double kG, double kS) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    ffModel = new ElevatorFeedforward(kS, kG, kV);
  }
}
