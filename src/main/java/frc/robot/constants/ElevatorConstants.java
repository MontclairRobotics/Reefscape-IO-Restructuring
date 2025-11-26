package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

  // Configs for TalonFX motors
  public static final Slot0Configs slot0Configs =
      new Slot0Configs()
          .withKP(1.4973)
          .withKI(0)
          .withKD(0.098147)
          .withKS(0.058548)
          .withKV(0.10758)
          .withKA(0.0013553)
          .withKG(0.22)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  public static final double METERS_PER_ROTATION = 36 * 5 / 1000.0 * (1.0 / 9);
  public static final double ROTATIONS_PER_METER = 1.0 / METERS_PER_ROTATION;

  // 0.9718607; // (meters) - distance between top bar and ground
  public static final double STARTING_HEIGHT = 0.98;

  public static final double MAX_EXTENSION = 1.22;
  public static final double MAX_ROTATIONS = MAX_EXTENSION * ROTATIONS_PER_METER;

  // distance between top bar and ground (fully extended)
  public static final double MAX_HEIGHT = MAX_EXTENSION + STARTING_HEIGHT;

  // The percent at the top and bottom of the elevator out of the
  // height of the elevator extention in which the elevator will slow
  // down to avoid crashing during manual control
  public static final double SLOW_DOWN_ZONE = 7.0;

  // (IN VOLTAGE) The lowest speed the elevator will go when it
  // thinks it is all the way at the top or bottom of the elevator
  // and is trying to go farther but has not yet hit the limit switch
  // during manual control
  public static final double SLOWEST_SPEED_VOLTS = 0.5;

  // motor constraints
  public static final double MAX_VELOCITY_RPS = 100;
  public static final double MAX_ACCEL_RPS = 350;

  public static final double PULLEY_RADIUS = Units.inchesToMeters(0.9175);
  public static final double SIM_GEARING = 12; // 12 spins of motor shaft to 1 spin of pulley

  public static final double STAGE2_MAX_HEIGHT = STARTING_HEIGHT + Units.inchesToMeters(23.743);

  public static final double MASS = Units.lbsToKilograms(40);

  public static final double RAISE_TIME = 2;

  public static final double VISUALIZATION_MIN_HEIGHT = 1.0; // In canvas units
  public static final double VISUALIZATION_MAX_HEIGHT = 3.0; // In canvas units

  public static final int LEFT_MOTOR_ID = 20;
  public static final int RIGHT_MOTOR_ID = 21;
}
