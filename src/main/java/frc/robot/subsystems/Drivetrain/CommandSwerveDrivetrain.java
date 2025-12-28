package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.PoseUtils;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

  private static final double kSimLoopPeriod = 0.002; // 2 ms
  private Notifier m_simNotifier = null;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

  private Rotation2d targetHeading = Rotation2d.fromDegrees(0);

  public AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  /* Heading PID Controller for things like automatic alignment buttons */
  public PIDController thetaController = new PIDController(5, 0, .1);

  /* variable to store our heading */
  public Rotation2d odometryHeading = new Rotation2d();

  private boolean isRobotAtAngleSetPoint; // for angle turning
  private boolean fieldRelative;

  public RobotConfig config;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
    if (Utils.isSimulation()) {
      startSimThread();
    }

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {

    }

    fieldRelative = true;
    configureAutoBuilder();
    thetaController.setTolerance(Degrees.of(1).in(Radians));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public static Rotation2d wrapAngle(Rotation2d ang) {
    double angle = ang.getDegrees();
    angle = (angle + 180) % 360; // Step 1 and 2
    if (angle < 0) {
      angle += 360; // Make sure it's positive
    }
    return Rotation2d.fromDegrees(angle - 180); // Step 3
  }

  public Rotation2d getWrappedHeading() {
    return wrapAngle(odometryHeading);
  }

  public double getVelocityXFromController() {
    double xInput = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.06);
    return Math.pow(xInput, 3) * DriveConstants.MAX_SPEED;
  }

  public double getVelocityYFromController() {
    double yInput = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.06);
    return Math.pow(yInput, 3) * DriveConstants.MAX_SPEED;
  }

  public void driveJoystick() {
    double rotInput = -MathUtil.applyDeadband(RobotContainer.driverController.getRightX(), 0.06);
    double rotVelocity = Math.pow(rotInput, 3) * DriveConstants.MAX_ANGULAR_SPEED;
    drive(
        getVelocityYFromController(),
        getVelocityXFromController(),
        rotVelocity,
        fieldRelative,
        true); // drives
  }

  public void drive(
      ChassisSpeeds speeds, boolean fieldRelative, boolean respectOperatorPerspective) {
    drive(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        fieldRelative,
        respectOperatorPerspective);
  }

  public void drive(
      double xSpeed,
      double ySpeed,
      double thetaSpeed,
      boolean fieldRelative,
      boolean respectOperatorPerspective) {

    if (respectOperatorPerspective) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red
          && fieldRelative) {
        xSpeed *= -1;
        ySpeed *= -1;
      }
    }

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getWrappedHeading());
    }

    SwerveRequest req =
        new SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(speeds)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);

    setControl(req);
  }

  public void setRobotRelativeAngle(Rotation2d angDeg) {
    double wrappedSetPoint = wrapAngle(odometryHeading.plus(angDeg)).getRadians();
    thetaController.setSetpoint(wrappedSetPoint);
  }

  public void alignToAngleRobotRelative(boolean lockDrive) {
    double response = thetaController.calculate(odometryHeading.getRadians());
    if (lockDrive) {
      drive(0, 0, response, false, true); // perspective doesn't matter in robot relative
    } else {
      drive(getVelocityYFromController(), getVelocityXFromController(), response, false, true);
    }
  }

  public void setFieldRelativeAngle(Rotation2d angle) {
    double wrappedAngle = wrapAngle(angle).getRadians();
    thetaController.setSetpoint(wrappedAngle);
  }

  public void alignToAngleFieldRelative(boolean lockDrive) {
    double response = thetaController.calculate(odometryHeading.getRadians());
    if (lockDrive) {
      drive(0, 0, response, true, true);
    } else {
      drive(getVelocityYFromController(), getVelocityXFromController(), response, true, true);
    }
  }

  public void alignToAngleRobotRelativeContinuous(
      Supplier<Rotation2d> angleSup, boolean lockDrive) {
    setRobotRelativeAngle(angleSup.get());
    alignToAngleRobotRelative(lockDrive);
  }

  public void toRobotRelative() {
    fieldRelative = false;
  }

  public void toFieldRelative() {
    fieldRelative = true;
  }

  @AutoLogOutput
  public Pose2d getRobotPose() {
    return this.getState().Pose;
  }

  public void zeroGyro() {
    resetRotation(PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)));
  }

  public Command driveJoystickInputCommand() {
    return Commands.run(() -> driveJoystick(), this);
  }

  public Command zeroGyroCommand() {
    return Commands.runOnce(() -> zeroGyro(), RobotContainer.drivetrain);
  }

  public Command toRobotRelativeCommand() {
    return Commands.runOnce(() -> toRobotRelative(), RobotContainer.drivetrain);
  }

  public Command toFieldRelativeCommand() {
    return Commands.runOnce(() -> toFieldRelative(), RobotContainer.drivetrain);
  }

  public Command alignToAngleRobotRelativeCommand(Rotation2d angle, boolean lockDrive) {
    return Commands.sequence(
        Commands.runOnce(() -> setRobotRelativeAngle(angle), RobotContainer.drivetrain),
        Commands.run(() -> alignToAngleRobotRelative(lockDrive), RobotContainer.drivetrain)
            .until(() -> isRobotAtAngleSetPoint));
  }

  public Command alignToAngleRobotRelativeContinuousCommand(
      Supplier<Rotation2d> angle, boolean lockDrive) {
    return Commands.run(() -> alignToAngleRobotRelativeContinuous(angle, lockDrive), this)
        .until(() -> isRobotAtAngleSetPoint);
  }

  public Command alignToAngleFieldRelativeCommand(Rotation2d angle, boolean lockDrive) {
    return Commands.sequence(
        Commands.runOnce(() -> setFieldRelativeAngle(angle), RobotContainer.drivetrain),
        Commands.run(() -> alignToAngleFieldRelative(lockDrive), this)
            .until(() -> isRobotAtAngleSetPoint));
  }

  @Override
  public void periodic() {

    odometryHeading = getRobotPose().getRotation();
    fieldRelative = !RobotContainer.driverController.L2().getAsBoolean();
    Logger.recordOutput("DriveState/FieldRelative", fieldRelative);

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }

    DogLog.log("BatteryVoltage", RobotController.getBatteryVoltage());
    DogLog.log("Drive/OdometryPose", getState().Pose);
    DogLog.log("Drive/TargetStates", getState().ModuleTargets);
    DogLog.log("Drive/MeasuredStates", getState().ModuleStates);
    DogLog.log("Drive/MeasuredSpeeds", getState().Speeds);
    if (mapleSimSwerveDrivetrain != null)
      DogLog.log(
          "Drive/SimulationPose",
          mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
  }

  public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

  @SuppressWarnings("unchecked")
  private void startSimThread() {
    mapleSimSwerveDrivetrain =
        new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            // TODO: modify the following constants according to your robot
            Pounds.of(150), // robot weight
            Inches.of(35.5), // bumper length
            Inches.of(35.5), // bumper width
            DCMotor.getKrakenX60Foc(1), // drive motor type
            DCMotor.getKrakenX60Foc(1), // steer motor type
            1.2, // wheel COF
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void resetPose(Pose2d pose) {
    if (this.mapleSimSwerveDrivetrain != null)
      mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
    Timer.delay(0.1); // wait for simulation to update
    super.resetPose(pose);
  }
}
