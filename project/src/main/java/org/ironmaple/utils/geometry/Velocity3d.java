package org.ironmaple.utils.geometry;

import java.util.Objects;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;

public class Velocity3d implements Interpolatable<Velocity3d> {
  private final double m_vx;
  private final double m_vy;
  private final double m_vz;

  /** Constructs a Velocity3d with X, Y, and Z components equal to zero. */
  public Velocity3d() {
    this(0.0, 0.0, 0.0);
  }

  /**
   * Constructs a Velocity3d with the X, Y, and Z components equal to the provided values.
   *
   * @param vx The x component of the velocity.
   * @param vy The y component of the velocity.
   * @param vz The z component of the velocity.
   */
  public Velocity3d(double vx, double vy, double vz) {
    m_vx = vx;
    m_vy = vy;
    m_vz = vz;
  }

  /**
   * Constructs a Velocity3d with the X, Y, and Z components equal to the provided values. The
   * components will be converted to and tracked as meters.
   *
   * @param x The x component of the velocity.
   * @param y The y component of the velocity.
   * @param z The z component of the velocity.
   */
  public Velocity3d(LinearVelocity x, LinearVelocity y, LinearVelocity z) {
    this(x.in(MetersPerSecond), y.in(MetersPerSecond), z.in(MetersPerSecond));
  }

  /**
   * Constructs a Velocity3d from the provided velocity vector's X, Y, and Z components. The
   * values are assumed to be in meters.
   *
   * @param vector The velocity vector to represent.
   */
  public Velocity3d(Vector<N3> vector) {
    this(vector.get(0), vector.get(1), vector.get(2));
  }

  /**
   * Constructs a Velocity3d with the provided speed and angle.
   *
   * @param speed The speed of the velocity.
   * @param angle The angle between the x-axis and the velocity vector.
   */
  public Velocity3d(double speed, Rotation3d angle) {
    final var rectangular = new Velocity3d(speed, 0.0, 0.0).rotateBy(angle);
    m_vx = rectangular.m_vx;
    m_vy = rectangular.m_vy;
    m_vz = rectangular.m_vz;
  }

  public Velocity3d(Velocity2d velocity2d) {
    this(velocity2d.getVX(), velocity2d.getVY(), 0.0);
  }

  /**
   * Returns the X component of the velocity.
   *
   * @return The X component of the velocity.
   */
  public double getVX() {
    return m_vx;
  }

  /**
   * Returns the Y component of the velocity.
   *
   * @return The Y component of the velocity.
   */
  public double getVY() {
    return m_vy;
  }

  /**
   * Returns the Z component of the velocity.
   *
   * @return The Z component of the velocity.
   */
  public double getVZ() {
    return m_vz;
  }

  /**
   * Returns a vector representation of this velocity.
   *
   * @return A Vector representation of this velocity.
   */
  public Vector<N3> toVector() {
    return VecBuilder.fill(m_vx, m_vy, m_vz);
  }

  /**
   * Returns the speed, or magnitude, of the velocity.
   *
   * @return The speed of the velocity.
   */
  public double getSpeed() {
    return Math.sqrt(m_vx * m_vx + m_vy * m_vy + m_vz * m_vz);
  }

  /**
   * Applies a rotation to the velocity in 3D space.
   *
   * <p>For example, rotating a Velocity3d of &lt;2, 0, 0&gt; by 90 degrees around the Z axis
   * will return a Velocity3d of &lt;0, 2, 0&gt;.
   *
   * @param other The rotation to rotate the velocity by.
   * @return The new rotated velocity.
   */
  public Velocity3d rotateBy(Rotation3d other) {
    final var p = new Quaternion(0.0, m_vx, m_vy, m_vz);
    final var qprime = other.getQuaternion().times(p).times(other.getQuaternion().inverse());
    return new Velocity3d(qprime.getX(), qprime.getY(), qprime.getZ());
  }

  /**
   * Returns the sum of two velocitys in 3D space.
   *
   * <p>For example, Velocity3d(1.0, 2.5, 3.5) + Velocity3d(2.0, 5.5, 7.5) =
   * Velocity3d{3.0, 8.0, 11.0).
   *
   * @param other The velocity to add.
   * @return The sum of the velocitys.
   */
  public Velocity3d plus(Velocity3d other) {
    return new Velocity3d(m_vx + other.m_vx, m_vy + other.m_vy, m_vz + other.m_vz);
  }

  /**
   * Returns the difference between two velocitys.
   *
   * <p>For example, Velocity3d(5.0, 4.0, 3.0) - Velocity3d(1.0, 2.0, 3.0) =
   * Velocity3d(4.0, 2.0, 0.0).
   *
   * @param other The velocity to subtract.
   * @return The difference between the two velocitys.
   */
  public Velocity3d minus(Velocity3d other) {
    return new Velocity3d(m_vx - other.m_vx, m_vy - other.m_vy, m_vz - other.m_vz);
  }

  /**
   * Returns the inverse of the current velocity. This is equivalent to negating all components
   * of the velocity.
   *
   * @return The inverse of the current velocity.
   */
  public Velocity3d unaryMinus() {
    return new Velocity3d(-m_vx, -m_vy, -m_vz);
  }

  /**
   * Returns the velocity multiplied by a scalar.
   *
   * <p>For example, Velocity3d(2.0, 2.5, 4.5) * 2 = Velocity3d(4.0, 5.0, 9.0).
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled velocity.
   */
  public Velocity3d times(double scalar) {
    return new Velocity3d(m_vx * scalar, m_vy * scalar, m_vz * scalar);
  }

  /**
   * Returns the velocity divided by a scalar.
   *
   * <p>For example, Velocity3d(2.0, 2.5, 4.5) / 2 = Velocity3d(1.0, 1.25, 2.25).
   *
   * @param scalar The scalar to multiply by.
   * @return The reference to the new mutated object.
   */
  public Velocity3d div(double scalar) {
    return new Velocity3d(m_vx / scalar, m_vy / scalar, m_vz / scalar);
  }

  public Velocity2d toVelocity2d() {
    return new Velocity2d(m_vx, m_vy);
  }

  @Override
  public String toString() {
    return String.format("Velocity3d(X: %.2f, Y: %.2f, Z: %.2f)", m_vx, m_vy, m_vz);
  }

  /**
   * Checks equality between this Velocity3d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Velocity3d) {
      return Math.abs(((Velocity3d) obj).m_vx - m_vx) < 1E-9
          && Math.abs(((Velocity3d) obj).m_vy - m_vy) < 1E-9
          && Math.abs(((Velocity3d) obj).m_vz - m_vz) < 1E-9;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_vx, m_vy, m_vz);
  }

  @Override
  public Velocity3d interpolate(Velocity3d endValue, double t) {
    return new Velocity3d(
        MathUtil.interpolate(this.getVX(), endValue.getVX(), t),
        MathUtil.interpolate(this.getVY(), endValue.getVY(), t),
        MathUtil.interpolate(this.getVZ(), endValue.getVZ(), t));
  }
}
