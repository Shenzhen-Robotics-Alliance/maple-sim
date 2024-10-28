package org.ironmaple.utils;

import edu.wpi.first.math.geometry.Translation3d;

public class ProjectileUtil {

    /**
     * Takes the time since last update, the current position, and the current velocity of a projectile
     * and returns the new velocity of the projectile.
     * 
     * <p>This can be used to implement custom drag, lift, or other forces on a projectile.
     * 
     * @param dt The time since the last update.
     * @param velocity The current velocity of the projectile.
     * @return The new velocity of the projectile.
     */
    @FunctionalInterface
    public interface ProjectileDynamics {
        /**
         * Calculate the new position of the projectile.
         * @param dt The time since the last update.
         * @param velocity The current velocity of the projectile.
         * @return The new velocity of the projectile.
         */
        Translation3d calculate(double dt, Translation3d velocity);
    }

    /**
     * Returns a {@link ProjectileDynamics} that applies gravity to the projectile.
     * 
     * @param g The acceleration due to gravity.
     * @return A {@link ProjectileDynamics} that applies gravity to the projectile.
     */
    public static ProjectileDynamics gravity(double g) {
        return (dt, velocity) -> {
            double verticalDeceleration = -g * dt;
            return new Translation3d(
                velocity.getX(),
                velocity.getY(),
                velocity.getZ() + verticalDeceleration
            );
        };
    }
}
