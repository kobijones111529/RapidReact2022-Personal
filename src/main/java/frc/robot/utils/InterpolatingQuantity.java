package frc.robot.utils;

import edu.wpi.first.math.interpolation.Interpolatable;
import org.jetbrains.annotations.NotNull;

import javax.measure.Quantity;

public class InterpolatingQuantity<T extends Quantity<T>> implements Interpolatable<InterpolatingQuantity<T>>, InverseInterpolatable<InterpolatingQuantity<T>>, Comparable<InterpolatingQuantity<T>> {
  public Quantity<T> value;

  public InterpolatingQuantity(Quantity<T> quantity) {
    this.value = quantity;
  }

  @Override
  public InterpolatingQuantity<T> interpolate(InterpolatingQuantity<T> endValue, double t) {
    Quantity<T> range = endValue.value.subtract(value);
    Quantity<T> y = range.multiply(t).add(value);
    return new InterpolatingQuantity<>(y);
  }

  @Override
  public double inverseInterpolate(InterpolatingQuantity<T> upper, InterpolatingQuantity<T> query) {
    Quantity<T> range = upper.value.subtract(value);
    if (range.getValue().doubleValue() < 0) return 0;
    Quantity<T> queryRelative = query.value.subtract(value);
    if (queryRelative.getValue().doubleValue() <= 0) return 0;
    return queryRelative.divide(range).getValue().doubleValue();
  }

  @Override
  public int compareTo(@NotNull InterpolatingQuantity<T> o) {
    return Double.compare(value.toSystemUnit().getValue().doubleValue(), o.value.toSystemUnit().getValue().doubleValue());
  }
}
