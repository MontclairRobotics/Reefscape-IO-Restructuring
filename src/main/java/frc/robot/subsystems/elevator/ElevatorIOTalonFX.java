package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {

    private TalonFX leftTalonFX;
    private TalonFX rightTalonFX;

    private MotionMagicVoltage mm_req;
    private Slot0Configs slot0Configs;
    private ElevatorFeedforward elevatorFeedforward;
    private SlewRateLimiter accelerationLimiter;

    private double setpoint = 0;

    public ElevatorIOTalonFX() {

        // instantiates Elevator motors
        leftTalonFX = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "Drivetrain");
        rightTalonFX = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "Drivetrain");

        // current limits to be applied
        CurrentLimitsConfigs currentLimitConfigs =
            new CurrentLimitsConfigs().withStatorCurrentLimit(80).withSupplyCurrentLimit(40);

        slot0Configs = ElevatorConstants.slot0Configs;
        // Motor configurations with slot0configs and current limit
        TalonFXConfiguration leftElevatorConfigs =
            new TalonFXConfiguration().withSlot0(slot0Configs).withCurrentLimits(currentLimitConfigs);

        TalonFXConfiguration rightElevatorConfigs =
            new TalonFXConfiguration().withSlot0(slot0Configs).withCurrentLimits(currentLimitConfigs);

        // Sets MotionMagic constraints
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity =
            ElevatorConstants.MAX_VELOCITY_RPS; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration =
            ElevatorConstants.MAX_ACCEL_RPS; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // Applies MotionMagic configs to motor configs
        leftElevatorConfigs.MotionMagic = motionMagicConfigs;
        rightElevatorConfigs.MotionMagic = motionMagicConfigs;

        // Applies motor configs to motors
        leftTalonFX.getConfigurator().apply(leftElevatorConfigs);
        rightTalonFX.getConfigurator().apply(rightElevatorConfigs);

        // Right TalonFX does whatever the left TalonFX does
        rightTalonFX.setControl(new Follower(ElevatorConstants.LEFT_MOTOR_ID, true));

        // Brake mode
        leftTalonFX.setNeutralMode(NeutralModeValue.Brake);
        rightTalonFX.setNeutralMode(NeutralModeValue.Brake);

        // Zeroes elevator upon initialization
        leftTalonFX.setPosition(0);
        rightTalonFX.setPosition(0);

        mm_req = new MotionMagicVoltage(0);
        accelerationLimiter = new SlewRateLimiter(5); 


        elevatorFeedforward = new ElevatorFeedforward(slot0Configs.kS, slot0Configs.kG, slot0Configs.kV);
    }

    public double calculateExtensionMeters() {
        double leftDisplacement = leftTalonFX.getPosition().refresh().getValueAsDouble();
        double rightDisplacement = rightTalonFX.getPosition().refresh().getValueAsDouble();
        return (leftDisplacement + rightDisplacement) / 2.0 * ElevatorConstants.METERS_PER_ROTATION;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.extensionMeters = calculateExtensionMeters();
        inputs.velocityMetersPerSec =
            leftTalonFX.getVelocity().refresh().getValueAsDouble()
                * ElevatorConstants.METERS_PER_ROTATION;

        inputs.leftPositionRot = leftTalonFX.getPosition().refresh().getValueAsDouble();
        inputs.rightPositionRot = rightTalonFX.getPosition().refresh().getValueAsDouble();

        inputs.leftAppliedVoltage = leftTalonFX.getMotorVoltage().refresh().getValueAsDouble();
        inputs.rightAppliedVoltage = rightTalonFX.getMotorVoltage().refresh().getValueAsDouble();

        inputs.leftTempCelsius = leftTalonFX.getDeviceTemp().refresh().getValueAsDouble();
        inputs.rightTempCelsius = rightTalonFX.getDeviceTemp().refresh().getValueAsDouble();

        inputs.leftCurrent = leftTalonFX.getStatorCurrent().refresh().getValueAsDouble();
        inputs.rightCurrent = rightTalonFX.getStatorCurrent().refresh().getValueAsDouble();
    }

    /**
     * Sets the elevator motors to a given position in rotations
     *
     * @param rotations the number of rotations to set the elevator motors to
     */
    @Override
    public void setPosition(double rotations) {
        rotations = MathUtil.clamp(rotations, 0, ElevatorConstants.MAX_ROTATIONS);
        leftTalonFX.setControl(mm_req.withPosition(rotations).withSlot(0).withEnableFOC(true));
    }

    @Override
    public void setVoltage(double voltage) {
        leftTalonFX.setControl(new VoltageOut(voltage).withEnableFOC(true));
    }

    @Override
    public void stop() {
        leftTalonFX.setControl(new VoltageOut(0));
    }

    
    @Override 
    public void resetEncoders(double rotationValue) {
        leftTalonFX.setPosition(rotationValue);
        rightTalonFX.setPosition(rotationValue);
    }

    @Override
    public void setCurrentLimits(double limit) {
        CurrentLimitsConfigs c = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(limit)
            .withSupplyCurrentLimit(40);
        leftTalonFX.getConfigurator().apply(c);
        rightTalonFX.getConfigurator().apply(c);
    }

    @Override
    public double stationaryElevatorFFVoltage() {
        return elevatorFeedforward.calculate(0, 0);
    }

    @Override 
    public void resetAccelLimiter() {
        accelerationLimiter.reset(0);
    }

    @Override
    public void setClimbGains() {
        leftTalonFX.getConfigurator().apply(new Slot0Configs().withKP(1.4973 * 3).withKI(0).withKD(0.098147)
        .withKS(0.058548).withKV(0.10758).withKA(0.0013553).withKG(0.22)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
        rightTalonFX.getConfigurator().apply(new Slot0Configs().withKP(1.4973 * 3).withKI(0).withKD(0.098147)
        .withKS(0.058548).withKV(0.10758).withKA(0.0013553).withKG(0.22)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
    }
}
