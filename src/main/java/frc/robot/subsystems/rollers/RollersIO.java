package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public double leftCurrent = 0;
    public double rightCurrent = 0;

    public double leftVelocity = 0;
    public double rightVelocity = 0;
    
    public double leftAppliedVoltage = 0;
    public double rightAppliedVoltage = 0;
  }

  /**
   * Update the IOInputs structure
   *
   * @param inputs IO Inputs to update
   */
  public default void updateInputs(RollersIOInputs inputs) {}

  /**
   * Applies voltages to each intake motor
   *
   * @param leftVoltage voltage to send to the LEFT intake motor
   * @param rightVoltage voltage to send to the RIGHT intake motor
   */
  public default void setVoltage(double leftVoltage, double rightVoltage) {}

  /**
   * Sets a voltage to both the left and right intake motor
   *
   * @param voltage the voltage to be applied
   */
  public default void setVoltage(double voltage) {}

  /**
   * Applies motor output to each intake motor, represented by a double from -1 to 1
   *
   * @param left output to the LEFT motor
   * @param right output to the RIGHT motor
   */
  public default void set(double left, double right) {}

  /**
   * Applies motor output to both intake motors, represented by a double from -1 to 1
   *
   * @param output output to both motors
   */
  public default void set(double output) {}

  /** Stops both intake motors */
  public default void stop() {}
}
