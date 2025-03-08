package frc.robot.subsystems.flywheels.genericFlywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class GenericFlywheelIOKracken implements GenericFlywheelIO {
  TalonFX flywheelMotor;

  private final StatusSignal<Angle> flywheelPosition;
  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;
  private final StatusSignal<Temperature> flywheelTemp;

  public GenericFlywheelIOKracken(int id, String canbus) {
    flywheelMotor = new TalonFX(id, canbus);

    flywheelPosition = flywheelMotor.getPosition();
    flywheelVelocity = flywheelMotor.getVelocity();
    flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
    flywheelCurrent = flywheelMotor.getStatorCurrent();
    flywheelTemp = flywheelMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelPosition,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelCurrent,
        flywheelTemp);
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    inputs.motorConnected = flywheelMotor.isConnected();
    inputs.voltage = flywheelPosition.getValueAsDouble();
    inputs.current = flywheelCurrent.getValueAsDouble();
    inputs.position = flywheelPosition.getValueAsDouble();
    inputs.veloctiy = flywheelVelocity.getValueAsDouble();
    inputs.tempC = flywheelTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    flywheelMotor.setControl(new VoltageOut(voltage));
  }
}
