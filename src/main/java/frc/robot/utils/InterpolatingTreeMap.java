package frc.robot.utils;

import edu.wpi.first.math.interpolation.Interpolatable;

import java.util.TreeMap;

public class InterpolatingTreeMap<K extends InverseInterpolatable<K> & Comparable<K>, V extends Interpolatable<V>> extends TreeMap<K, V> {
  private final int maxSize;

  public InterpolatingTreeMap(int maxSize) {
    this.maxSize = maxSize;
  }

  public InterpolatingTreeMap() {
    this(0);
  }

  @Override
  public V put(K key, V value) {
    if (maxSize > 0 && maxSize <= size()) {
      remove(firstKey());
    }
    return super.put(key, value);
  }

  public V getInterpolated(K key) {
    V value = get(key);
    if (value == null) {
      K lowerKey = floorKey(key);
      K upperKey = ceilingKey(key);

      if (lowerKey == null && upperKey == null) {
        return null;
      }
      else if (lowerKey == null) {
        return get(upperKey);
      }
      else if (upperKey == null) {
        return get(lowerKey);
      }

      V lowerValue = get(lowerKey);
      V upperValue = get(upperKey);
      return lowerValue.interpolate(upperValue, lowerKey.inverseInterpolate(upperKey, key));
    }
    else {
      return value;
    }
  }
}
