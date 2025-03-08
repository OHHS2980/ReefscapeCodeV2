package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOKracken implements ClimberIO {
  TalonFX motor;

  // Inputs from drive motor
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  public ClimberIOKracken() {
    motor = new TalonFX(39, "canivore");

    motorPosition = motor.getPosition();
    driveVelocity = motor.getVelocity();
    driveAppliedVolts = motor.getMotorVoltage();
    driveCurrent = motor.getStatorCurrent();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.voltage = driveAppliedVolts.getValueAsDouble();
    inputs.current = driveCurrent.getValueAsDouble();
    inputs.position = motorPosition.getValueAsDouble();
    inputs.veloctiy = driveVelocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(new VoltageOut(voltage));
  }
}
