package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ArmConstants;
import frc.robot.util.PoseUtils;

public class ArmIOSim implements ArmIO {

    private DutyCycleEncoderSim encoder;
    private SingleJointedArmSim sim;

    private PIDController pidController;

    private Rotation2d setpoint;
    private double appliedVoltage;

    public ArmIOSim() {
        pidController = new PIDController(43.555, 0, 0.9555);

        encoder.setConnected(true);

        sim =
            new SingleJointedArmSim(
                DCMotor.getNEO(1),
                ArmConstants.ARM_TO_MOTOR,
                10,
                0,
                ArmConstants.ARM_MIN_ANGLE.getRadians(),
                ArmConstants.ARM_MAX_ANGLE.getRadians(),
                true,
                0,
                null);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.appliedVoltage = appliedVoltage;
        inputs.current = sim.getCurrentDrawAmps();

        inputs.armAngle = getArmAngle();
        inputs.elbowAngle = getElbowAngle();
        inputs.forearmAngle = getForearmAngle();

        inputs.encoderConnected = encoder.getConnected();
        inputs.forearmSetpoint = setpoint;
    }

    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        sim.setInputVoltage(voltage);
        encoder.set(sim.getAngleRads());
    }

    public void setForearmAngle(Rotation2d angle) {
        angle.minus(getElbowAngle());
        setpoint = angle;

        double target =
            MathUtil.clamp(
                angle.getRotations(),
                ArmConstants.ARM_MIN_ANGLE.getRotations(),
                ArmConstants.ARM_MAX_ANGLE.getRotations());

        double voltage = pidController.calculate(getArmAngle().getRotations(), target) * 5;
        voltage = MathUtil.clamp(voltage, -ArmConstants.VOLTAGE_LIMIT, ArmConstants.VOLTAGE_LIMIT);
        setVoltage(-voltage);
        encoder.set(sim.getAngleRads());
    }

    public void stop() {
        sim.setInputVoltage(0);
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
        return ArmIOSparkMax.wrapAngle(Rotation2d.fromRadians(encoder.get()));
    }

    public boolean atSetpoint() {
        return false;
    }

    public double getPercentRotation() {
        double distance = PoseUtils.getAngleDistance(getForearmAngle(), ArmConstants.ARM_MIN_ANGLE).getDegrees();
        double interval = PoseUtils.getAngleDistance(ArmConstants.ARM_MAX_ANGLE, ArmConstants.ARM_MIN_ANGLE).getDegrees();
        return distance / interval;
    }   

    public void setIdleMode(IdleMode mode) {}

    public void resetPIDController() {
        pidController.reset();
    }
}
