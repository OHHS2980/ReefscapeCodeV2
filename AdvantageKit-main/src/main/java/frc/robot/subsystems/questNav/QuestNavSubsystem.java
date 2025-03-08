package frc.robot.subsystems.questNav;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNavIO io;
  QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

  public QuestNavSubsystem(QuestNavIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("QuestNav", inputs);

    // Add addQuestObservation to robot state
  }
}
