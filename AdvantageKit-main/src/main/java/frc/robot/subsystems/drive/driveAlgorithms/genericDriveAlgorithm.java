package frc.robot.subsystems.drive.driveAlgorithms;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface genericDriveAlgorithm {
  public default SwerveModuleState[] calculateSwerveStates() {
    SwerveModuleState[] result = new SwerveModuleState[4];

    for (int i = 0; i < result.length; i++) {
      result[i] = new SwerveModuleState();
    }

    return result;
  }
}
