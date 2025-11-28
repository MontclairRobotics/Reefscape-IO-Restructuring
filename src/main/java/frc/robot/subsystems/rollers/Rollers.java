package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RollerConstants;
import frc.robot.util.GamePiece;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  private final Debouncer isStalledDebouncer = new Debouncer(0.05, DebounceType.kRising);

  private GamePiece heldPiece = GamePiece.Coral;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public GamePiece getHeldPiece() {
    return heldPiece;
  }

  public boolean hasCoral() {
    return heldPiece == GamePiece.Coral;
  }

  public boolean hasAlgae() {
    return heldPiece == GamePiece.Algae;
  }

  public boolean isStalled() {
    return isStalledDebouncer.calculate(
        inputs.leftCurrent > RollerConstants.ROLLER_STALL_CURRENT
            || inputs.rightCurrent > RollerConstants.ROLLER_STALL_CURRENT);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
  }

  // -------------------- COMMANDS --------------------

  public Command stopCommand() {
    return Commands.runOnce(() -> io.stop(), this);
  }

  public Command intakeAlgaeCommand() {
    return Commands.run(() -> io.set(RollerConstants.ALGAE_INTAKE_SPEED), this)
        .until(this::isStalled)
        .finallyDo(
            () -> {
              io.stop();
              heldPiece = GamePiece.Algae;
            });
  }

  public Command outtakeAlgaeCommand() {
    return Commands.run(() -> io.set(RollerConstants.ALGAE_OUTTAKE_SPEED), this)
        .withTimeout(2)
        .finallyDo(
            () -> {
              io.stop();
              heldPiece = GamePiece.None;
            });
  }

  public Command intakeCoralCommand() {
    return Commands.run(() -> io.set(RollerConstants.CORAL_INTAKE_SPEED), this)
        .until(this::isStalled)
        .finallyDo(
            () -> {
              io.stop();
              heldPiece = GamePiece.Coral;
            });
  }

  public Command outtakeCoralCommand() {
    return Commands.run(() -> io.set(RollerConstants.CORAL_OUTTAKE_SPEED), this)
        .withTimeout(2)
        .finallyDo(
            () -> {
              io.stop();
              heldPiece = GamePiece.None;
              // RobotContainer.leds.playLEDPattern(LEDs.blink(Color.kYellow), 0.2);
            });
  }

  public Command intakeCoralJiggleCommand() {
    return Commands.run(() -> io.set(RollerConstants.CORAL_INTAKE_SPEED), this)
        .until(this::isStalled)
        .andThen(
            Commands.sequence(
                Commands.run(() -> io.set(-0.1), this)
                    .withTimeout(0.1)
                    .andThen(intakeCoralCommand())))
        .andThen(
            Commands.sequence(
                Commands.run(() -> io.set(-0.1), this)
                    .withTimeout(0.1)
                    .andThen(intakeCoralCommand())))
        .finallyDo(
            () -> {
              this.heldPiece = GamePiece.Coral;
            });
  }

  public Command holdCoralCommand() {
    return Commands.run(() -> io.set(RollerConstants.CORAL_HOLDING_SPEED), this);
  }

  @Override
  public Command getDefaultCommand() {
    return Commands.run(
        () -> {
          if (hasCoral()) io.set(RollerConstants.CORAL_HOLDING_SPEED);
          if (hasAlgae()) io.set(RollerConstants.ALGAE_HOLDING_SPEED);
        },
        this);
  }
}
