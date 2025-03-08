package frc.robot.subsystems.flywheels.genericFlywheel;

import static frc.robot.util.SparkUtil.sparkStickyFault;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GenericFlywheelIONeo implements GenericFlywheelIO {
  SparkMax flywheelMotor;
  SparkMaxConfig flywheelConfig;
  SparkClosedLoopController flywheelController;

  public GenericFlywheelIONeo(int canId, SparkMaxConfig flywheelConfig) {
    flywheelMotor = new SparkMax(canId, MotorType.kBrushless);
    this.flywheelConfig = flywheelConfig;

    flywheelMotor.configure(
        flywheelConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    flywheelController = flywheelMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    inputs.voltage = 0.0;
    inputs.current = 0.0;
    inputs.position = 0.0;
    inputs.veloctiy = 0.0;
    inputs.tempC = 0.0;

    inputs.motorConnected = !sparkStickyFault;
  }

  @Override
  public void setVoltage(double voltage) {
    flywheelMotor.setVoltage(voltage);
  }
}
