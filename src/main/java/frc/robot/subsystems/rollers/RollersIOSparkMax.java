package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.rollers.RollersIO.RollersIOInputs;

public class RollersIOSparkMax implements RollersIO {

  private SparkMax leftMotor;
  private SparkMax rightMotor;
  SparkMaxConfig config;

  public RollersIOSparkMax() {
    // instantiate motor objects
    leftMotor = new SparkMax(RollerConstants.LEFT_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(RollerConstants.RIGHT_ID, MotorType.kBrushless);

    // create config
    config = new SparkMaxConfig();
    config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);

    // configure motors
    rightMotor.configure(
        config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(
        config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.leftCurrent = leftMotor.getOutputCurrent();
    inputs.rightCurrent = rightMotor.getOutputCurrent();
    inputs.leftAppliedVoltage = leftMotor.getAppliedOutput();
    inputs.rightAppliedVoltage = rightMotor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double leftVoltage, double rightVoltage) {
    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }

  @Override
  public void setVoltage(double voltage) {
    setVoltage(voltage, voltage);
  }

  @Override
  public void set(double left, double right) {
    leftMotor.set(left);
    rightMotor.set(right);
  }

  @Override
  public void set(double output) {
    set(output, output);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
