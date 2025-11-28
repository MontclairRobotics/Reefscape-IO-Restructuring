package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ArmConstants {

  public static final int DEVICE_ID = 29;
  public static final double VOLTAGE_LIMIT = 1.7555; // 1.80555;
  public static final double MAX_VELOCITY = 60.0 / 360.0; // rotations per sec
  public static final double MAX_ACCELERATION = 20.0 / 360.0; // rotations per sec per sec

  public static final double ARM_TO_MOTOR =
      25 * 1.5; // for every rotation of the arm the motor moves this much

  public static final Rotation2d SHOULDER_ENCODER_OFFSET = Rotation2d.fromDegrees(-83 + 3.5);
  public static final Rotation2d ARM_MAX_ANGLE =
      Rotation2d.fromDegrees(34); // TODO: use protractor to get this for the real robot
  public static final Rotation2d ARM_MIN_ANGLE =
      Rotation2d.fromDegrees(-56); // TODO: use protractor to get this for the real robot

  // The max safe angle of the endpoint to the horizontal
  public static final Rotation2d FOREARM_MAX_ANGLE = Rotation2d.fromDegrees(43);

  // The min safe angle of the endpoint to the horizontal
  public static final Rotation2d FOREARM_MIN_ANGLE = Rotation2d.fromDegrees(-(180 - 102.143));

  // Angle of endpoint is -37.8
  public static final double ARM_ANGLE_TO_FOREARM_ANGLE_RATIO = 30.0 / 14.0;
  public static final Rotation2d FOREARM_ANGLE_WHEN_ARM_IS_HORIZONTAL =
      Rotation2d.fromDegrees(-34.903);
  public static final double J1Length = 0.19 - Units.inchesToMeters(2);
}
