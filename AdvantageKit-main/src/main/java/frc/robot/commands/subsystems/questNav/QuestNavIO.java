package frc.robot.subsystems.questNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
  @AutoLog
  class QuestNavIOInputs {
    public boolean connected = false;
    public Pose2d questPose = new Pose2d();
    public double timestamp = 0.0;
    public double batteryPercent = 0.0;
    public Quaternion questQuaternion = new Quaternion();
  }

  // Updates inputs
  public default void updateInputs(QuestNavIOInputs inputs) {}

  // Zero the relativerobot heading
  public default void zeroHeading() {}

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public default void zeroPosition() {}

  // Clean up questnav subroutine messages after processing on the headset
  public default void cleanUpQuestNavMessages() {}
}
