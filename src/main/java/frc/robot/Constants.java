// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

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

  public class DriveConstants {

    public static final Distance bumperWidth = Inches.of(30);
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(10).in(MetersPerSecondPerSecond);
    public static final double MAX_ANGULAR_SPEED = RotationsPerSecond.of(2).in(RadiansPerSecond);
    public static final double MAX_ANGULAR_ACCELERATION =
        RotationsPerSecond.of(3).in(RadiansPerSecond);

    public static final PathConstraints DEFAULT_CONSTRAINTS =
        new PathConstraints(
            MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);
  }

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

    public static final int SUPPLY_CURRENT_LIMIT = 40; // amps
  }

  public class RollerConstants {

    // CAN IDs
    public static final int LEFT_ID = 30;
    public static final int RIGHT_ID = 31;

    // speeds for intaking and outtaking various game pieces
    public static final double CORAL_INTAKE_SPEED = 0.5;
    public static final double CORAL_OUTTAKE_SPEED = -1;
    public static final double ALGAE_INTAKE_SPEED = 0.2;
    public static final double ALGAE_OUTTAKE_SPEED = -1;

    // current we expect the motors to stall at when a coral gets stuck
    public static final double ROLLER_STALL_CURRENT = 22;
    public static final double CORAL_HOLDING_SPEED = 0.1;
    public static final double ALGAE_HOLDING_SPEED = 0.1;
  }
}
