package frc.robot.subsystems.flywheels.genericFlywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class GenericFlywheelIOSim implements GenericFlywheelIO {
  DCMotor gearbox = DCMotor.getNeo550(1);

  private DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.5, 25), gearbox);

  private double appliedVoltage = 0.0;

  public GenericFlywheelIOSim() {}

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    // Update sim
    appliedVoltage = MathUtil.clamp(appliedVoltage, -12, 12);

    sim.setInput(appliedVoltage);
    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
    inputs.voltage = appliedVoltage;
    inputs.current = sim.getCurrentDrawAmps();
    inputs.position = sim.getAngularPositionRad();
    inputs.veloctiy = sim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
  }
}
