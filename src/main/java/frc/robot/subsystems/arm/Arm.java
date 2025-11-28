package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.RobotState;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.RotatedRect;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Arm extends SubsystemBase {

    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private RobotState targetState;

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Rollers", inputs);
    }

    public boolean atSetpoint() {
        return io.atSetpoint();
    }

    public void stop() {
        io.stop();
    }

    public void setTargetState(RobotState state) {
        this.targetState = state;
    }

    public RobotState getTargetState() {
        return targetState;
    }

    public void resetPIDController() {
        io.resetPIDController();
    }

    public double getPercentRotation() {
        return io.getPercentRotation();
    }

    public void setIdleMode(IdleMode mode) {
        io.setIdleMode(mode);
    }

    public Rotation2d getArmAngle() {
        return io.getArmAngle();
    }

    public Rotation2d getElbowAngle() {
        return io.getElbowAngle();
    }
    
    public Rotation2d getForearmAngle() {
        return io.getForearmAngle();
    }

    public void setForearmAngle(Rotation2d angle) {
        io.setForearmAngle(angle);
    }

    public void joystickControl() {
        double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3) * 12;
        voltage += io.calculateStationaryFeedforward();
        voltage = MathUtil.clamp(voltage, -3, 3);
        io.setVoltage(voltage);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public Command goToAngleCommand(Rotation2d angle) {
        return Commands.run(() -> {
            setForearmAngle(angle);
        }, this)
        .until(this::atSetpoint)
        .finallyDo(() -> {
            stop();
            resetPIDController();
        });
    }

    public Command goToAngleContinuousCommand(Rotation2d angle) {
        return Commands.run(() -> {
            setForearmAngle(angle);
        }, this)
        .finallyDo(() -> {
            stop();
            resetPIDController();
        });
    }

    public Command joystickControlCommand() {
        return Commands.run(this::joystickControl, this);
    }

    public Command setIdleModeCommand(IdleMode mode) {
        return Commands.runOnce(() -> setIdleMode(mode))
        .ignoringDisable(true);
    }

    public Command setState(RobotState state) {
        return goToAngleCommand(state.getAngle());
    }

    public Command holdState(RobotState state) {
        return goToAngleContinuousCommand(state.getAngle());
    }

}

