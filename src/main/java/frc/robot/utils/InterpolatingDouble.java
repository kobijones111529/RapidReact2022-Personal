package frc.robot.utils;

import edu.wpi.first.math.interpolation.Interpolatable;
import org.jetbrains.annotations.NotNull;

public class InterpolatingDouble implements Interpolatable<InterpolatingDouble>, InverseInterpolatable<InterpolatingDouble>, Comparable<InterpolatingDouble> {
  public double value;

  public InterpolatingDouble(double value) {
    this.value = value;
  }

  @Override
  public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
    double range = other.value - value;
    double y = range * x + value;
    return new InterpolatingDouble(y);
  }

  @Override
  public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
    double range = upper.value - value;
    if (range < 0) return 0;
    double queryRelative = query.value - value;
    if (queryRelative <= 0) return 0;
    return queryRelative / range;
  }

  @Override
  public int compareTo(@NotNull InterpolatingDouble o) {
    return Double.compare(value, o.value);
  }
}
