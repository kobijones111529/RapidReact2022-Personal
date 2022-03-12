package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import si.uom.SI;
import systems.uom.common.USCustomary;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;
import javax.measure.quantity.Angle;
import javax.measure.quantity.Length;
import java.util.Optional;

public final class LimelightUtil {
  private static final String HAS_TARGET_KEY = "tv";
  private static final String X_KEY = "tx";
  private static final String Y_KEY = "ty";

  private LimelightUtil() {

  }

  public static boolean hasTarget(String table) {
    return hasTarget(NetworkTableInstance.getDefault().getTable(table));
  }

  public static boolean hasTarget(NetworkTable table) {
    return table.getEntry(HAS_TARGET_KEY).getBoolean(false);
  }

  public static Optional<Quantity<Angle>> getTargetXOffset(String table) {
    return getTargetXOffset(NetworkTableInstance.getDefault().getTable(table));
  }

  public static Optional<Quantity<Angle>> getTargetXOffset(NetworkTable table) {
    if (!hasTarget(table)) {
      return Optional.empty();
    }

    return Optional.of(Quantities.getQuantity(table.getEntry(X_KEY).getDouble(0), USCustomary.DEGREE_ANGLE));
  }

  public static Optional<Quantity<Length>> getDistanceToTarget(String table, Quantity<Angle> cameraAngle, Quantity<Length> targetHeight) {
    return getDistanceToTarget(NetworkTableInstance.getDefault().getTable(table), cameraAngle, targetHeight);
  }

  public static Optional<Quantity<Length>> getDistanceToTarget(NetworkTable table, Quantity<Angle> cameraAngle, Quantity<Length> targetHeight) {
    if (!hasTarget(table)) {
      return Optional.empty();
    }

    Quantity<Angle> cameraToTarget = Quantities.getQuantity(table.getEntry(X_KEY).getDouble(0), USCustomary.DEGREE_ANGLE);
    Quantity<Angle> angleToTarget = cameraAngle.add(cameraToTarget);

    // tan(θ) = y / x
    Quantity<Length> distance = targetHeight.divide(Math.tan(angleToTarget.to(SI.RADIAN).getValue().doubleValue()));

    return Optional.of(distance);
  }
}
