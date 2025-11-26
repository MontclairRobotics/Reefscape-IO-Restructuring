package frc.robot.subsystems.rollers;

public class RollersIOSim implements RollersIO {

    // Internal simulated motor state
  private double leftAppliedVoltage = 0.0;
  private double rightAppliedVoltage = 0.0;

  private double leftVelocity = 0.0;
  private double rightVelocity = 0.0;

  private double leftCurrent = 0.0;
  private double rightCurrent = 0.0;

  // Simple constants for simulation
  private static final double FREE_SPEED_RPM = 2000;   // pretend motor max speed

  public RollersIOSim() {}

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    // Convert applied volts → fake velocity
    // 12V → max speed
    leftVelocity = (leftAppliedVoltage / 12.0) * FREE_SPEED_RPM;
    rightVelocity = (rightAppliedVoltage / 12.0) * FREE_SPEED_RPM;

    // Fake linear current model
    leftCurrent = Math.abs(leftAppliedVoltage / 12.0) * Rollers.ROLLER_STALL_CURRENT;
    rightCurrent = Math.abs(rightAppliedVoltage / 12.0) * Rollers.ROLLER_STALL_CURRENT;

    // Populate IOInputs
    inputs.leftAppliedVoltage = leftAppliedVoltage;
    inputs.rightAppliedVoltage = rightAppliedVoltage;

    inputs.leftVelocity = leftVelocity;
    inputs.rightVelocity = rightVelocity;

    inputs.leftCurrent = leftCurrent;
    inputs.rightCurrent = rightCurrent;

  }

  @Override
  public void setVoltage(double left, double right) {
    leftAppliedVoltage = left;
    rightAppliedVoltage = right;
  }

}
