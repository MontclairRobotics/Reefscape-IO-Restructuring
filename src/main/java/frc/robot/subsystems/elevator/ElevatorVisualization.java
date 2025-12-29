package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;

public class ElevatorVisualization {

  private Pose3d stage2Pose;
  private Pose3d stage3Pose;

  public void update(ElevatorIOInputs inputs) {
    stage2Pose =
        new Pose3d(
            -0.103,
            0,
            0.14
                + Math.max(
                    0,
                    inputs.extensionMeters
                        + ElevatorConstants.STARTING_HEIGHT
                        - ElevatorConstants.STAGE2_MAX_HEIGHT),
            Rotation3d.kZero);
    stage3Pose = new Pose3d(-0.103, 0, 0.165 + inputs.extensionMeters, Rotation3d.kZero);
  }

  public void log() {
    Logger.recordOutput("Elevator/Stage2Pose", stage2Pose);
    Logger.recordOutput("Elevator/Stage3Pose", stage3Pose);
  }
}
