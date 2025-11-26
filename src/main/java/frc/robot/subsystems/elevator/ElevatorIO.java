package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public class ElevatorIOInputs {
    public double extensionMeters = 0;
    public double velocityMetersPerSec = 0;

    public double leftPositionRot = 0;
    public double rightPositionRot = 0;

    public double leftAppliedVoltage = 0;
    public double rightAppliedVoltage = 0;

    public double leftTempCelsius = 0;
    public double rightTempCelsius = 0;

    public double leftCurrent = 0;
    public double rightCurrent = 0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(double rotations) {}

  public default void setVoltage(double voltage) {}

  public default void stop() {}

  public default boolean atSetpoint() {return false;}

  public default void resetEncoders(double rotationValue) {}

  public default void setCurrentLimits(double limit) {}

  public default double stationaryElevatorFFVoltage() {return 0;}

  public default void resetAccelLimiter() {}

  public default void setClimbGains() {}
}
