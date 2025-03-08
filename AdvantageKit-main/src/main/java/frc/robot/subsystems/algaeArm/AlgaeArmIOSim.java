package frc.robot.subsystems.algaeArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class AlgaeArmIOSim implements AlgaeArmIO {
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(DCMotor.getNEO(2), 25, 1, 0.5, 0, Math.PI / 2, false, 0);
  private PIDController pid = new PIDController(0, 0, 0);
  private ArmFeedforward ffModel = new ArmFeedforward(0, 0, 0, 0);

  private boolean closedLoop = false;
  private double ffVoltage = 0;
  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(AlgaeArmIOInputs inputs) {
    // Update sim
    if (closedLoop) {
      appliedVoltage = pid.calculate(sim.getAngleRads()) + ffVoltage;
    }

    appliedVoltage = MathUtil.clamp(appliedVoltage, -12, 12);
    sim.setInput(appliedVoltage);
    sim.update(Constants.loopPeriodSecs);

    // Update inputs
    inputs.armPositionRad = sim.getAngleRads();
    inputs.armVelocityRadPerSec = sim.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    appliedVoltage = voltage;
  }

  @Override
  public void setSetpoint(double positionRad, double velocityRadPerSec) {
    closedLoop = true;
    pid.setSetpoint(positionRad);
    this.ffVoltage = ffModel.calculate(sim.getAngleRads(), velocityRadPerSec);
  }

  @Override
  public void configurePID(
      double kP, double kI, double kD, double kV, double kA, double kG, double kS) {
    pid.setPID(kP, kI, kD);
    ffModel = new ArmFeedforward(kS, kG, kV, kA);
  }
}
