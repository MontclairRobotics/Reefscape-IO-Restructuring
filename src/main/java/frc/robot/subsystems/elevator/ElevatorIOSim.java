package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {

  // represents elevator motors
  private final DCMotor elevatorMotors = DCMotor.getKrakenX60Foc(2);

  // keeps track of the voltage applied
  private double appliedVoltage = 0;

  private SlewRateLimiter accelerationLimiter;

  // for calculating accel
  double lastTime = Timer.getFPGATimestamp();
  double lastVelocity;

  // PID + FF
  private ProfiledPIDController pidController;
  public ElevatorFeedforward elevatorFeedforward;

  // for modeling elevator movement
  private ElevatorSim sim =
      new ElevatorSim(
          elevatorMotors,
          12, // 12 spins motor to 1 spin pulley
          ElevatorConstants.MASS,
          ElevatorConstants.PULLEY_RADIUS,
          0.0,
          ElevatorConstants.MAX_EXTENSION,
          true,
          0.0);

  public ElevatorIOSim() {
    // creates feedforward
    elevatorFeedforward =
        new ElevatorFeedforward(
            ElevatorConstants.slot0Configs.kS,
            ElevatorConstants.slot0Configs.kG,
            ElevatorConstants.slot0Configs.kV);

    // accel limiter for joystick control
    accelerationLimiter = new SlewRateLimiter(5);

    // creates pid controller
    pidController =
        new ProfiledPIDController(
            8.2697,
            0,
            0.068398,
            new Constraints(ElevatorConstants.MAX_VELOCITY_RPS, ElevatorConstants.MAX_ACCEL_RPS));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02); // iterate elevator sim
    // update inputs
    inputs.extensionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.leftAppliedVoltage = appliedVoltage;
    inputs.rightAppliedVoltage = appliedVoltage;
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage; // logs voltage
    sim.setInputVoltage(voltage); // sets voltage to sim
  }

  @Override
  public void setPosition(double rotations) {
    double goalRotations =
        MathUtil.clamp(rotations, 0, ElevatorConstants.MAX_ROTATIONS); // clamps rotations
    double pidOutputVoltage =
        pidController.calculate(
            sim.getPositionMeters() * ElevatorConstants.ROTATIONS_PER_METER, goalRotations);

    double accel =
        (pidController.getSetpoint().velocity * ElevatorConstants.METERS_PER_ROTATION
                - lastVelocity)
            / (Timer.getFPGATimestamp() - lastTime);

    lastTime = Timer.getFPGATimestamp();
    lastVelocity = sim.getVelocityMetersPerSecond();

    double ffVoltage = elevatorFeedforward.calculate(pidController.getSetpoint().velocity, accel);

    double outputVoltage = MathUtil.clamp(pidOutputVoltage + ffVoltage, -12, 12);

    appliedVoltage = outputVoltage; // logs voltage
    sim.setInputVoltage(outputVoltage); // sets voltage to sim
  }

  public void stop() {
    setVoltage(0);
  }

  public void resetEncoders(double rotationValue) {
    sim.setState(0, 0); // TODO: is this correct?
  }

  public double stationaryElevatorFFVoltage() {
    return elevatorFeedforward.calculate(0, 0);
  }

  public void resetAccelLimiter() {
    accelerationLimiter.reset(0);
  }

  @Override
  public void setClimbGains() {
    pidController.setPID(1.4973 * 3, 0, 0.098147);
    elevatorFeedforward.setKa(0.0013553);
    elevatorFeedforward.setKg(0.22);
    elevatorFeedforward.setKs(0.058548);
    elevatorFeedforward.setKv(0.10758);
  }
}
