package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {

  // represents elevator motors
  private final DCMotor elevatorMotors = DCMotor.getKrakenX60Foc(2);

  // keeps track of the voltage applied
  private double appliedVoltage = 0;

  // for calculating accel
  double lastTime = Timer.getFPGATimestamp();
  double lastVelocity;

  // PID + FF
  private ProfiledPIDController pidController;
  public ElevatorFeedforward elevatorFeedforward;

  // Visualization
  private Mechanism2d mechanism;
  private MechanismRoot2d rootMechanism;
  private MechanismLigament2d elevatorMechanism;

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

    // creates pid controller
    pidController =
        new ProfiledPIDController(
            8.2697,
            0,
            0.068398,
            new Constraints(ElevatorConstants.MAX_VELOCITY_RPS, ElevatorConstants.MAX_ACCEL_RPS));

    // Visualization
    mechanism = new Mechanism2d(4, 4);
    rootMechanism = mechanism.getRoot("ElevatorBottom", 2, 0);
    elevatorMechanism =
        rootMechanism.append(
            new MechanismLigament2d("Elevator", ElevatorConstants.STARTING_HEIGHT, 90));
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

    // for calculating FF
    double accel =
        (pidController.getSetpoint().velocity * ElevatorConstants.METERS_PER_ROTATION
                - lastVelocity)
            / (Timer.getFPGATimestamp() - lastTime);

    // Updates values for calculating accel
    lastTime = Timer.getFPGATimestamp();
    lastVelocity = sim.getVelocityMetersPerSecond();

    // FF
    double ffVoltage = elevatorFeedforward.calculate(pidController.getSetpoint().velocity, accel);

    // Adds FF + PID voltages
    double outputVoltage = MathUtil.clamp(pidOutputVoltage + ffVoltage, -12, 12);

    appliedVoltage = outputVoltage; // logs voltage
    sim.setInputVoltage(outputVoltage); // sets voltage to sim
  }
}
