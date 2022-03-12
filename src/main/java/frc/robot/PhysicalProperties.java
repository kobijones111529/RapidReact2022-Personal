package frc.robot;

import edu.wpi.first.math.Pair;
import si.uom.SI;
import si.uom.quantity.AngularSpeed;
import si.uom.quantity.Density;
import systems.uom.common.USCustomary;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;
import javax.measure.quantity.Angle;
import javax.measure.quantity.Length;
import javax.measure.quantity.Speed;
import java.util.Optional;

public final class PhysicalProperties {
  private static final double SHOOTER_RPM_TO_INITIAL_BALL_MPS_SCALAR = 1; // TODO find value
  private static final Quantity<Density> DENSITY_OF_AIR = Quantities.getQuantity(1.225, CustomUnits.KILOGRAM_PER_CUBIC_METRE);


  private PhysicalProperties() {

  }

  public static Pair<Double, Double> calculateVelocity(double time, double mass, double diameter, double initialVelX, double initialVelY) {
    double gravity = -9.8;
    double density = 1.225;
    double c = (Math.PI / 16) * density * (diameter * diameter);

    double dt = 0.0001;

    double velX = initialVelX;
    double velY = initialVelY;

    for (double t = 0; t < time; t += dt) {
      // F = mg - cv
      double dragX = -(c * velX);
      double dragY = (mass * gravity) - (c * velY);

      double accelX = dragX / mass;
      double accelY = dragY / mass;

      velX -= accelX * dt;
      velY -= accelY * dt;
    }

    return new Pair<>(velX, velY);
  }

  public static Pair<Double, Double> calculatePosition(double time, double mass, double diameter, double initialVelX, double initialVelY, double initialPosX, double initialPosY) {
    double gravity = -9.8;
    double density = 1.225;
    double c = (Math.PI / 16) * density * (diameter * diameter);

    double dt = 0.0001;

    double velX = initialVelX;
    double velY = initialVelY;

    double posX = initialPosX;
    double posY = initialPosY;

    for (double t = 0; t < time; t += dt) {
      // F = mg - cv
      double dragX = -(c * velX);
      double dragY = (mass * gravity) - (c * velY);

      double accelX = dragX / mass;
      double accelY = dragY / mass;

      velX += accelX * dt;
      velY += accelY * dt;

      posX += velX * dt;
      posY = velY * dt;
    }

    return new Pair<>(posX, posY);
  }

  public static Quantity<Angle> getShootingAngle() {
    double angleRadians = 0; // TODO find value
    return Quantities.getQuantity(angleRadians, SI.RADIAN);
  }

  public static Quantity<Speed> getInitialBallSpeed(Quantity<AngularSpeed> shooterSpeed) {
    double shooterSpeedRPM = shooterSpeed.to(USCustomary.REVOLUTION_PER_MINUTE).getValue().doubleValue();
    return Quantities.getQuantity(SHOOTER_RPM_TO_INITIAL_BALL_MPS_SCALAR * shooterSpeedRPM, SI.METRE_PER_SECOND);
  }

  public static Quantity<AngularSpeed> getShooterSpeed(Quantity<Speed> initialBallSpeed) {
    double initialBallSpeedMetersPerSecond = initialBallSpeed.to(SI.METRE_PER_SECOND).getValue().doubleValue();
    return Quantities.getQuantity((1 / SHOOTER_RPM_TO_INITIAL_BALL_MPS_SCALAR) * initialBallSpeedMetersPerSecond, USCustomary.REVOLUTION_PER_MINUTE);
  }

  public static Optional<Quantity<Speed>> getIdealShooterVelocity(Quantity<Angle> shooterAngle, Quantity<Length> targetDistance, Quantity<Length> targetHeight) {
    double gravityMetersPerSecond = Constants.GRAVITY.to(SI.METRE_PER_SQUARE_SECOND).getValue().doubleValue();
    double shooterAngleRadians = shooterAngle.to(SI.RADIAN).getValue().doubleValue();
    double targetDistanceMeters = targetDistance.to(SI.METRE).getValue().doubleValue();
    double targetHeightMeters = targetHeight.to(SI.METRE).getValue().doubleValue();

    double radicand = (gravityMetersPerSecond * Math.pow(targetDistanceMeters, 2)) / (2 * Math.pow(Math.cos(shooterAngleRadians), 2) * (targetHeightMeters - targetDistanceMeters * Math.tan(shooterAngleRadians)));
    if (radicand <= 0) {
      return Optional.empty();
    }

    double shooterVelocityMetersPerSecond = Math.sqrt(radicand);

    return Optional.of(Quantities.getQuantity(shooterVelocityMetersPerSecond, SI.METRE_PER_SECOND));
  }
}
