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

  public void updateInputs(ElevatorIOInputs inputs);

  public void setPosition(double rotations);

  public void setVoltage(double voltage);

  public void stop();

  public void resetEncoders(double rotationValue);

  public double stationaryElevatorFFVoltage();

  public void resetAccelLimiter();

  public void setClimbGains();
}
