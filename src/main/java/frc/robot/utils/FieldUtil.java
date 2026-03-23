package frc.robot.utils;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class FieldUtil {
  // newton's method constants
  private static final int maxIter = 10;
  private static final LinearVelocity projectileHorizontalVelocity = MetersPerSecond.of(100000000);
  private static final double E_tolerance = 0.1;
  private static final double dT_dt_tolerance =
      SwerveConstants.driverTranslationalShootingVelocity.in(MetersPerSecond)
          * Math.cos(Math.toRadians(20)) // min overlap angle at worst-case robot velocity
          / projectileHorizontalVelocity.in(MetersPerSecond);

  private static double prevSwitchTime = 110; // starts at first transition shift

  /** Gets the alliance from the DS. If the alliance can't be retreived, blue is used by default. */
  public static Alliance getAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return alliance.get();
    }

    return Alliance.Blue;
  }

  public static boolean hubActive() {
    if (DriverStation.isAutonomous() || DriverStation.getMatchTime() <= 30) {
      return true;
    }

    String data = DriverStation.getGameSpecificMessage();
    boolean active = false;

    switch (getAlliance()) {
      case Blue:
        active = data.charAt(0) == 'B';
      case Red:
        active = data.charAt(0) == 'R';
    }

    if (prevSwitchTime - 25 >= DriverStation.getMatchTime()) {
      active = !active;
      prevSwitchTime = DriverStation.getMatchTime();
    }

    return active;
  }

  /** Whether the supplied robot pose is in the bump zone(s). */
  public static boolean inBumpZone(Pose2d robotPose) {
    if (FieldConstants.blueBumpZone.contains(robotPose.getTranslation())
        || FieldConstants.redBumpZone.contains(robotPose.getTranslation())) {
      return true;
    }

    return false;
  }

  /** Whether the supplied robot pose is in the ferry zone, depending on alliance. */
  public static boolean inFerryZone(Pose2d robotPose) {
    if (getAlliance() == Alliance.Blue) {
      return robotPose.getX() > FieldConstants.ferryXThresholdBlue;
    }

    return robotPose.getX() < FieldConstants.ferryXThresholdRed;
  }

  /** Gets the correct ferry target location for shooting. */
  public static Translation2d getFerryTarget(Pose2d robotPose) {
    if (getAlliance() == Alliance.Blue) {
      return robotPose.getY() > FieldConstants.ferryYThreshold
          ? FieldConstants.blueFerryTop
          : FieldConstants.blueFerryBottom;
    }

    return robotPose.getY() > FieldConstants.ferryYThreshold
        ? FieldConstants.redFerryTop
        : FieldConstants.redFerryBottom;
  }

  /** Gets the correct hub target location for shooting. */
  public static Translation2d getHubTarget() {
    return getAlliance() == Alliance.Blue ? FieldConstants.blueHub : FieldConstants.redHub;
  }

  /** Using the robot pose, finds the shot target location (hub / ferry). */
  public static Translation2d getShotTarget(Pose2d robotPose) {
    return inFerryZone(robotPose) ? getFerryTarget(robotPose) : getHubTarget();
  }

  /**
   * Finds the shot target at the current robot pose, then uses newton's method to update the
   * supplied shot parameters.
   */
}