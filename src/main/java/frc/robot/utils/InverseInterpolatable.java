package frc.robot.utils;

public interface InverseInterpolatable<T> {
  double inverseInterpolate(T upper, T query);
}
