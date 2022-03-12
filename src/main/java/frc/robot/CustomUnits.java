package frc.robot;

import si.uom.SI;
import si.uom.quantity.AngularSpeed;
import si.uom.quantity.Density;
import systems.uom.common.USCustomary;

import javax.measure.MetricPrefix;
import javax.measure.Unit;

public final class CustomUnits {
  public static final Unit<AngularSpeed> REVOLUTION_PER_100_MILLISECOND = USCustomary.REVOLUTION.divide(MetricPrefix.DECI(SI.SECOND)).asType(AngularSpeed.class);
  public static final Unit<Density> KILOGRAM_PER_CUBIC_METRE = SI.KILOGRAM.divide(SI.CUBIC_METRE).asType(Density.class);

  private CustomUnits() {

  }
}
