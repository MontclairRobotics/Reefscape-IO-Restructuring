package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.GamePiece;

public class Rollers extends SubsystemBase {

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

  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  private final Debouncer isStalledDebouncer = new Debouncer(0.05, DebounceType.kRising);
  private final Debouncer isHeldDebouncer = new Debouncer(0.1, DebounceType.kFalling);

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
            inputs.leftCurrent > ROLLER_STALL_CURRENT ||
            inputs.rightCurrent > ROLLER_STALL_CURRENT
        );
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
        return Commands.run(() -> io.set(ALGAE_INTAKE_SPEED), this)
            .until(this::isStalled)
            .finallyDo(() -> {
                io.stop();
                heldPiece = GamePiece.Algae;
            });
    }

    public Command outtakeAlgaeCommand() {
        return Commands.run(() -> io.set(ALGAE_OUTTAKE_SPEED), this)
            .withTimeout(2)
            .finallyDo(() -> {
                io.stop();
                heldPiece = GamePiece.None;
            });
    }

    public Command intakeCoralCommand() {
        return Commands.run(() -> io.set(CORAL_INTAKE_SPEED), this)
            .until(this::isStalled)
            .finallyDo(() -> {
                io.stop();
                heldPiece = GamePiece.Coral;
            });
    }

    public Command outtakeCoralCommand() {
        return Commands.run(() -> io.set(CORAL_OUTTAKE_SPEED), this)
            .withTimeout(2)
            .finallyDo(() -> {
                io.stop();
                heldPiece = GamePiece.None;
                //RobotContainer.leds.playLEDPattern(LEDs.blink(Color.kYellow), 0.2);
            });
    }

    public Command intakeCoralJiggleCommand() {
        return Commands.run(() -> io.set(CORAL_INTAKE_SPEED), this)
            .until(this::isStalled)
            .andThen(Commands.sequence(
                Commands.run(() -> io.set(-0.1), this)
                .withTimeout(0.1)
                .andThen(intakeCoralCommand())
            )).andThen(Commands.sequence(
                Commands.run(() -> io.set(-0.1), this)
                .withTimeout(0.1)
                .andThen(intakeCoralCommand())
            )).finallyDo(() -> {
                this.heldPiece = GamePiece.Coral;
            });
            //.until(this::hasPiece).finallyDo(() -> RobotContainer.leds.playLEDPattern(LEDs.blink(Color.kGreen), 1));
            
    }

    public Command holdCoralCommand() {
        return Commands.run(() -> io.set(CORAL_HOLDING_SPEED), this);
    }

    @Override
    public Command getDefaultCommand() {
        return Commands.run(() -> {
            if (hasCoral()) io.set(CORAL_HOLDING_SPEED);
            if (hasAlgae()) io.set(ALGAE_HOLDING_SPEED);
        }, this);
    }

}
