package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.RobotState;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  ElevatorIO
      io; // io object, is either an ElevatorIOSim or ElevatorIOTalonFX object depending on if we
  // are simulating or not
  private final ElevatorIOInputsAutoLogged inputs =
      new ElevatorIOInputsAutoLogged(); // inputs, auto logged

  private double setpoint = 0; // keeps track of setpoint

  private RobotState targetState =
      RobotState.L1; // keep tracks of what state the robot wants to go to

  public Elevator(ElevatorIO io) {
    this.io = io;
    setDefaultCommand(Commands.run(this::joystickControl));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /**
   * @param state the mechanism state to raise to
   * @return the amount of estimate time, in seconds, that it will take the elevator to reach its
   *     goal
   */
  public double getRaiseTime(RobotState state) {
    double percentExtension =
        Math.abs((state.getHeight() - inputs.extensionMeters) / ElevatorConstants.MAX_EXTENSION);
    return Math.pow(percentExtension, 0.3) + 0.8 * percentExtension + 1;
  }

  /**
   * Sets new stator current limit to both motors
   *
   * @param statorLimit the stator limit in amps, to apply
   */
  public void setCurrentLimit(double statorLimit) {
    io.setCurrentLimits(statorLimit);
  }

  public Command setCurrentLimitCommand(double limit) {
    return runOnce(() -> setCurrentLimit(limit));
  }

  /**
   * @return extension of elevator from 0, in meters
   */
  public double getExtension() {
    return inputs.extensionMeters;
  }

  /**
   * @return height of elevator, measured from ground to the top of the 3rd stage
   */
  public double getHeight() {
    return getExtension() + ElevatorConstants.STARTING_HEIGHT;
  }

  /**
   * @return velocity of both elevator motors
   */
  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  /**
   * @return whether or not the elevator is within 0.5 inches of our setpoint
   */
  public boolean atSetpoint() {
    return Math.abs(setpoint - getExtension()) < Units.inchesToMeters(0.5);
  }

  /**
   * @return measured position of the elevator motors, in rotations
   */
  public double getPosition() {
    return inputs.extensionMeters * ElevatorConstants.ROTATIONS_PER_METER;
  }

  /**
   * @return value from 0 to 1 representing how much the elevator has traveled relative to its max
   *     extension
   */
  public double percentExtension() {
    return getExtension() / ElevatorConstants.MAX_EXTENSION;
  }

  /**
   * @param voltage input voltage, pre-calculation
   * @return voltage to apply when in a slow down zone
   */
  public double calculateSlowDownVoltage(double voltage) {
    if (voltage < 0) {
      if (percentExtension() <= 0.01) {
        voltage = 0;
        io.resetAccelLimiter();
      } else if (percentExtension() <= 0.07) {
        voltage =
            Math.max(
                voltage,
                (-12
                        * Math.pow(
                            (percentExtension() * (100.0 / ElevatorConstants.SLOW_DOWN_ZONE)), 3.2))
                    - ElevatorConstants.SLOWEST_SPEED_VOLTS);
      }
    }
    if (voltage > 0) {
      if (percentExtension() >= 0.99) {
        voltage = 0;
        io.resetAccelLimiter();
        ;
      } else if (percentExtension() >= 0.93) {
        voltage =
            Math.min(
                voltage,
                (12
                        * Math.pow(
                            (percentExtension() * (100.0 / ElevatorConstants.SLOW_DOWN_ZONE)), 3.2))
                    + ElevatorConstants.SLOWEST_SPEED_VOLTS);
      }
    }
    return voltage;
  }

  /** Function to handle input and output with joysticks */
  public void joystickControl() {
    double voltage =
        Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.2), 3)
            * 12;

    voltage += io.stationaryElevatorFFVoltage();
    voltage = calculateSlowDownVoltage(voltage);
    voltage = MathUtil.clamp(voltage, -12, 12);

    io.setVoltage(voltage);
  }

  public void stop() {
    io.stop();
  }

  // SYSID
  public final SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 1 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("Elevator-State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> {
                io.setVoltage(volts.in(Volts));
              },
              null,
              this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  /**
   * resets elevator encoders to a specific value
   *
   * @param value the value in rotations to set the elevator's encoders to
   */
  public void resetEncoders(double value) {
    io.resetEncoders(value);
  }

  /**
   * Uses Motion Magic to bring the elevator to the desired position
   *
   * @param extension position to move to
   */
  public void setExtension(double extension) {
    setpoint = extension;
    io.setPosition(extension * ElevatorConstants.ROTATIONS_PER_METER);
  }

  public Command setExtensionCommand(double extension) {
    return Commands.run(() -> setExtension(extension), this).until(this::atSetpoint);
  }

  /**
   * @param state the Robot State we want the elevator to travel to
   * @return command to set this extension
   */
  public Command setState(RobotState state) {
    return Commands.run(
            () -> {
              setExtension(state.getHeight());
            },
            this)
        .until(this::atSetpoint)
        .finallyDo(this::stop);
  }

  public Command setTargetState(RobotState state) {
    return Commands.runOnce(
        () -> {
          targetState = state;
        });
  }

  public RobotState getTargetState() {
    return targetState;
  }
}
