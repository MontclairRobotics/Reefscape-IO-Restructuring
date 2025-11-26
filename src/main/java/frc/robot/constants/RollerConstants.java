package frc.robot.constants;

public class RollerConstants {

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
}
