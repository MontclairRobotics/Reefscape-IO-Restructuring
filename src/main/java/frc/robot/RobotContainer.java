// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparkMax;
import frc.robot.util.PoseUtils;
import frc.robot.util.RobotState;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Rollers rollers;
  public static Elevator elevator;
  public static Arm arm;
  public static CommandSwerveDrivetrain drivetrain;
  public static MapleSimSwerveDrivetrain mapleDrivetrain;

  private SwerveDriveSimulation driveSimulation = null;

  // Controllers
  public static final CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("unchecked")
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        rollers = new Rollers(new RollersIOSparkMax());
        elevator = new Elevator(new ElevatorIOTalonFX());
        arm = new Arm(new ArmIOSparkMax());

        drivetrain = TunerConstants.createDrivetrain();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        rollers = new Rollers(new RollersIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        drivetrain = TunerConstants.createDrivetrain();

        driveSimulation = drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive;

        break;

      default:
        // Replayed robot, disable IO implementations
        // NOTE: The Pearadox codebase leaves this field unset ?!
        // See
        // https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/RobotContainer.java#L168.
        rollers = new Rollers(null);
        elevator = new Elevator(null);
        arm = new Arm(null);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());

    // Robot relative
    driverController
        .L2()
        .onTrue(drivetrain.toRobotRelativeCommand())
        .onFalse(drivetrain.toFieldRelativeCommand());

    // 90 degree buttons
    driverController
        .triangle()
        .onTrue(
            drivetrain.alignToAngleFieldRelativeCommand(
                PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
    driverController
        .square()
        .onTrue(drivetrain.alignToAngleFieldRelativeCommand((Rotation2d.fromDegrees(-54)), false));
    driverController
        .cross()
        .onTrue(
            drivetrain.alignToAngleFieldRelativeCommand(
                PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
    driverController
        .circle()
        .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(54), false));

    // zeros gyro
    driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());

    // L1 Manual
    operatorController
        .cross()
        .and(operatorController.L2().negate())
        .whileTrue(arm.holdState(RobotState.L1))
        .onFalse(
            elevator
                .setState(RobotState.L1)
                .alongWith(elevator.setTargetState(RobotState.L1))
                .alongWith(arm.holdState(RobotState.L1)));

    // L2 Manual
    operatorController
        .square()
        .and(operatorController.L2().negate())
        .whileTrue(arm.holdState(RobotState.L2))
        .onFalse(
            elevator
                .setState(RobotState.L2)
                .alongWith(elevator.setTargetState(RobotState.L2))
                .alongWith(arm.holdState(RobotState.L2)));

    // L3 Manual
    operatorController
        .triangle()
        .and(operatorController.L2().negate())
        .whileTrue((arm.holdState(RobotState.L3)))
        .onFalse(
            elevator
                .setState(RobotState.L3)
                .alongWith(elevator.setTargetState(RobotState.L3))
                .alongWith(arm.holdState(RobotState.L3)));

    operatorController
        .touchpad()
        .onTrue(
            Commands.runOnce(
                () -> {
                  arm.setDefaultCommand(arm.joystickControlCommand());
                }));

    // L4 Manual
    operatorController
        .circle()
        .and(operatorController.L2().negate())
        .whileTrue(arm.holdState(RobotState.L4).alongWith(elevator.setState(RobotState.L3)))
        .onFalse(elevator.setState(RobotState.L4).alongWith(arm.holdState(RobotState.L4)));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Resets the simulation.
   *
   * <p>Borrowed from
   * https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/RobotContainer.java#L394.
   */
  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drivetrain.resetPose(new Pose2d(12, 2, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
    // AlgaeHandler.getInstance().reset();
  }

  /**
   * Updates Simulated Arena; to be called from Robot.simulationPeriodic()
   *
   * <p>Borrowed from
   * https://github.com/Pearadox/2025RobotCode/blob/main/src/main/java/frc/robot/RobotContainer.java#L402
   */
  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    // The pose by maplesim, including collisions with the field.
    // See https://www.chiefdelphi.com/t/simulated-robot-goes-through-walls-with-maplesim/508663.
    Logger.recordOutput(
        "FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
