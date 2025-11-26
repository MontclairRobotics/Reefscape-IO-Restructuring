package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.w3c.dom.css.ElementCSSInlineStyle;

import com.ctre.phoenix6.SignalLogger;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.RobotState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {

    ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private Debouncer velocityDebouncer = new Debouncer(0.1, DebounceType.kFalling);
    private double setpoint = 0;
    private RobotState targetState = RobotState.L1;

    SlewRateLimiter accelerationLimiter;

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
     * 
     * @param state the mechanism state to raise to
     * @return the amount of estimate time, in seconds, that it will take 
     * the elevator to reach its goal
     */
    public double getRaiseTime(RobotState state) {
        double percentExtension = Math.abs((state.getHeight() - inputs.extensionMeters) / ElevatorConstants.MAX_EXTENSION);
        return Math.pow(percentExtension, 0.3) + 0.8 * percentExtension + 1;
    }

    public void setCurrentLimit(double limit) {io.setCurrentLimits(limit);}
    
    public Command setCurrentLimitCommand(double limit) {
        return runOnce(() -> setCurrentLimit(limit));
    }

    public double getExtension() {
        return inputs.extensionMeters;
    }

    public double getHeight() {
        return getExtension() + ElevatorConstants.STARTING_HEIGHT;
    }

    public double getVelocity() {
        return inputs.velocityMetersPerSec;
    }

    public boolean isVelociatated() {
        return velocityDebouncer.calculate(Math.abs(inputs.velocityMetersPerSec) > 0.1);
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getExtension()) < Units.inchesToMeters(0.5);
    }

    public double getPosition() {
        return inputs.extensionMeters * ElevatorConstants.ROTATIONS_PER_METER;
    }

    public double percentExtension() {
        return getExtension() / ElevatorConstants.MAX_EXTENSION;
    }

    public double calculateSlowDownVoltage(double voltage) {
        if (voltage < 0) {
            if (percentExtension() <= 0.01) {
                voltage = 0;
                io.resetAccelLimiter();
            } else if (percentExtension() <= 0.07) {
                voltage = Math.max(voltage, (-12 * Math.pow((percentExtension() * (100.0 /
                        ElevatorConstants.SLOW_DOWN_ZONE)), 3.2)) - ElevatorConstants.SLOWEST_SPEED_VOLTS);
            }
        }
        if (voltage > 0) {
            if (percentExtension() >= 0.99) {
                voltage = 0;
                io.resetAccelLimiter();;
            } else if (percentExtension() >= 0.93) {
                voltage = Math.min(voltage, (12 * Math.pow((percentExtension() * (100.0 /
                        ElevatorConstants.SLOW_DOWN_ZONE)), 3.2)) + ElevatorConstants.SLOWEST_SPEED_VOLTS);
            }
        }
        return voltage;
    }

    public void joystickControl() {
        //getting raw voltage
        double voltage = Math.pow(-MathUtil.applyDeadband(RobotContainer.operatorController.getLeftY(), 0.2), 3) * 12;

        //voltage calculations + adjustments
        voltage += io.stationaryElevatorFFVoltage();
        voltage = calculateSlowDownVoltage(voltage);
        voltage = MathUtil.clamp(voltage, -12, 12);

        //setting voltage
        io.setVoltage(voltage);
    }

    public void stop() {
        io.stop();
    }
    
    public final SysIdRoutine routine = new SysIdRoutine(
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
            this)
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public void resetEncoders(double value) {
        io.resetEncoders(value);
    }

    public void setExtension(double extension) {
        setpoint = extension;
        io.setPosition(extension * ElevatorConstants.ROTATIONS_PER_METER);
    }

    public Command setExtensionCommand(double extension) {
        return Commands.run(() -> setExtension(extension), this)
            .until(this::atSetpoint);
    }

    public Command setState(RobotState state) {
        return Commands.run(() -> {
            setExtension(state.getHeight());
        }, this)
        .until(this::atSetpoint)
        .finallyDo(this::stop);
    }

    public Command setTargetState(RobotState state) {
        return Commands.runOnce(() -> {
            targetState = state;
        });
    }

    public RobotState getTargetState() {
        return targetState;
    }
}
