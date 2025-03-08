package frc.robot.subsystems.algaeArm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeArm extends SubsystemBase {
  private AlgaeArmIO io;
  private final AlgaeArmIOInputsAutoLogged inputs = new AlgaeArmIOInputsAutoLogged();
  private SysIdRoutine sysId;
  private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(0, 0));
  private Boolean auto = false;
  private State goalState = new State(0, 0);

  @RequiredArgsConstructor
  public enum ArmGoal {
    STOW(-90),
    INTAKE(-45);
    private final double armSetpoint;

    private double getRadians() {
      return Units.degreesToRadians(armSetpoint);
    }
  }

  @Getter @AutoLogOutput ArmGoal goal = ArmGoal.STOW;

  public AlgaeArm(AlgaeArmIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        io.configurePID(0, 0, 0.0, 0, 0, 0, 0);
        break;
      case SIM:
        io.configurePID(0.5, 0, 0, 0, 0, 0, 0);
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    Logger.recordOutput("Arm/Name", getName());
    Logger.recordOutput("Arm/Subsystem", getSubsystem());
    Logger.recordOutput("Arm/AutoMove", auto);
    Pose3d armOrigin = new Pose3d();
    Logger.recordOutput(
        "Arm/ArmPose",
        new Pose3d(
            new Translation3d()
                .minus(armOrigin.getTranslation())
                .rotateBy(new Rotation3d(0, inputs.armPositionRad, 0))
                .plus(armOrigin.getTranslation()),
            new Rotation3d(0, inputs.armPositionRad, 0)));

    if (auto) {
      goalState =
          profile.calculate(
              Constants.loopPeriodSecs,
              new State(inputs.armPositionRad, inputs.armVelocityRadPerSec),
              new State(goal.getRadians(), 0));
      io.setSetpoint(goalState.position, goalState.velocity);
    }
  }

  /** Run open loop at specified voltage */
  public void setVoltage(double voltage, boolean safe) {
    auto = false;
    if ((safe && (inputs.armPositionRad > Math.PI && voltage > 0)
        || (inputs.armPositionRad < 0 && voltage < 0))) {
      io.setVoltage(0);
    } else {
      io.setVoltage(voltage);
    }
  }

  public void setGoal(ArmGoal goal) {
    auto = true;
    this.goal = goal;
  }

  public Rotation2d getArmAngle() {
    return Rotation2d.fromRadians(inputs.armPositionRad);
  }

  @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(inputs.armPositionRad, goal.getRadians(), 1e-3);
  }
}
