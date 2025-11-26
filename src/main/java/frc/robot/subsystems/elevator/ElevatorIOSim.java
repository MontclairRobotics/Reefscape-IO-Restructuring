package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor elevatorMotors = DCMotor.getKrakenX60Foc(2);

  private double appliedVoltage = 0;

  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();
  double lastVelocity;

  private ProfiledPIDController pidController;
  public ElevatorFeedforward elevatorFeedforward;

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
    // create feedforward
    elevatorFeedforward =
        new ElevatorFeedforward(
            ElevatorConstants.slot0Configs.kS,
            ElevatorConstants.slot0Configs.kG,
            ElevatorConstants.slot0Configs.kV);

    // create pid controller
    pidController =
        new ProfiledPIDController(
            8.2697,
            0,
            0.068398,
            new Constraints(ElevatorConstants.MAX_VELOCITY_RPS, ElevatorConstants.MAX_ACCEL_RPS));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    inputs.extensionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.leftAppliedVoltage = appliedVoltage;
    inputs.rightAppliedVoltage = appliedVoltage;
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    sim.setInputVoltage(voltage);
  }

  @Override
  public void setPosition(double rotations) {
    if (rotations > ElevatorConstants.MAX_ROTATIONS) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "Setting the elevator height outside of range",
              "Somebody is messing up the button setting in robot container by setting the height to higher the range",
              5000));
    }
    if (rotations < 0) {
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              "Setting the elevator height outside of range",
              "Somebody is messing up the button setting in robot container by setting the height to lower than 0 (who is doing this???! WTF??)",
              5000));
    }

    double goalRotations = MathUtil.clamp(rotations, 0, ElevatorConstants.MAX_ROTATIONS);
    double pidOutputVoltage = pidController.calculate(sim.getPositionMeters() * ElevatorConstants.ROTATIONS_PER_METER, goalRotations);
    double accel = (pidController.getSetpoint().velocity * ElevatorConstants.METERS_PER_ROTATION - lastVelocity) / (Timer.getFPGATimestamp() - lastTime);

    lastTime = Timer.getFPGATimestamp();
    lastVelocity = sim.getVelocityMetersPerSecond();

    double ffVoltage = elevatorFeedforward.calculate(pidController.getSetpoint().velocity, accel);
    double outputVoltage = MathUtil.clamp(pidOutputVoltage + ffVoltage, -12, 12);

    appliedVoltage = outputVoltage;
    sim.setInputVoltage(outputVoltage);
  }
}
