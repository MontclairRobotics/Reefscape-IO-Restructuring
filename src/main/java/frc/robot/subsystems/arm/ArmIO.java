package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double appliedVoltage;
    public double current;
    public double tempCelsius;

    public Rotation2d armAngle;
    public Rotation2d elbowAngle;
    public Rotation2d forearmAngle;

    public boolean encoderConnected;
    public Rotation2d forearmSetpoint;
  }

  public void updateInputs(ArmIOInputs inputs);

  public void setVoltage(double voltage);

  public void setForearmAngle(Rotation2d angle);

  public double calculateStationaryFeedforward();

  public void stop();

  /**
   * @return the angle of the endpoint (second joint) relative to the horizontal
   */
  public Rotation2d getForearmAngle();
  
  /**
   * @return the angle between the arm and the forearm (the angle between the two joints)
   */
  public Rotation2d getElbowAngle();

  /**
   * @return the angle of the arm (joint 1) relative to the horizontal
   */
  public Rotation2d getArmAngle();

  public boolean atSetpoint();

  public double getPercentRotation();

  public void setIdleMode(IdleMode mode);

  public void resetPIDController();
}
