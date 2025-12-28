package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class AngleUtils {

  // yes, this is a class for this one method
  /** WRAPS THE ANGLE FROM -180 TO 180 DEGREES */
  public static Rotation2d wrapAngle(Rotation2d ang) {
    double angle = ang.getDegrees();
    angle = (angle + 180) % 360;
    if (angle < 0) {
      angle += 360; // Make sure it's positive
    }
    return Rotation2d.fromDegrees(angle - 180);
  }
}
