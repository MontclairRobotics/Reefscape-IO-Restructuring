package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.ArmConstants;
import frc.robot.util.PoseUtils;
import frc.robot.util.AngleUtils;

public class ArmIOSparkMax implements ArmIO {

    // motor controller
    private SparkMax motor;

    // pid + ff controlers
    private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.2, 0);
    private PIDController pidController = new PIDController(43.555, 0, 0.9555);

    // encoders
    private RelativeEncoder relativeEncoder;
    private DutyCycleEncoder armEncoder;

    // tracks setpoint
    private Rotation2d setpoint;

    public ArmIOSparkMax() {
        TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION);
        motor = new SparkMax(ArmConstants.DEVICE_ID, MotorType.kBrushless);

        armEncoder = new DutyCycleEncoder(5, 1, ArmConstants.SHOULDER_ENCODER_OFFSET.getRotations());
        armEncoder.setInverted(true);

        relativeEncoder = motor.getEncoder();
        relativeEncoder.setPosition(getElbowAngle().getRotations() * ArmConstants.ARM_TO_MOTOR);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(true).voltageCompensation(12);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void updateInputs(ArmIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput();
        inputs.current = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();

        inputs.armAngle = this.getArmAngle();
        inputs.elbowAngle = this.getElbowAngle();
        inputs.forearmAngle = this.getForearmAngle();

        inputs.encoderConnected = armEncoder.isConnected();
        inputs.forearmSetpoint = setpoint;
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setForearmAngle(Rotation2d angle) {
        setpoint = angle;

        double target =
            MathUtil.clamp(
                angle.getRotations(),
                ArmConstants.FOREARM_MIN_ANGLE.getRotations(),
                ArmConstants.FOREARM_MAX_ANGLE.getRotations()
            );

        double voltage = pidController.calculate(getForearmAngle().getRotations(), target) * 5;
        voltage = MathUtil.clamp(voltage, -ArmConstants.VOLTAGE_LIMIT, ArmConstants.VOLTAGE_LIMIT);
        setVoltage(-voltage);
    }

    public double calculateStationaryFeedforward() {
        double voltage = armFeedforward.calculate(getArmAngle().getRadians(), 0);
        return getArmAngle().getDegrees() > 0 ? voltage : -voltage;
    }

    public void stop() {
        motor.stopMotor();
    }

    public Rotation2d getForearmAngle() {
        return getArmAngle().plus(getElbowAngle());
    }

    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(
            (getArmAngle().getRotations() * (-ArmConstants.ARM_ANGLE_TO_FOREARM_ANGLE_RATIO))
                + ArmConstants.FOREARM_ANGLE_WHEN_ARM_IS_HORIZONTAL.getRotations());
    }

    public Rotation2d getArmAngle() {
        if (armEncoder.isConnected()) {
            return AngleUtils.wrapAngle(Rotation2d.fromRotations(armEncoder.get()));
        } else {
            return AngleUtils.wrapAngle(
                Rotation2d.fromRotations(
                    (relativeEncoder.getPosition() / ArmConstants.ARM_TO_MOTOR) % 1));
        }
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public double getPercentRotation() {
        double distance = PoseUtils.getAngleDistance(getForearmAngle(), ArmConstants.ARM_MIN_ANGLE).getDegrees();
        double interval = PoseUtils.getAngleDistance(ArmConstants.ARM_MAX_ANGLE, ArmConstants.ARM_MIN_ANGLE).getDegrees();
        return distance / interval;
    }   

    public void setIdleMode(IdleMode mode) {
        motor.configure(new SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void resetPIDController() {
        pidController.reset();
    }
}
