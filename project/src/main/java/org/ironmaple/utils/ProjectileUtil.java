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
     * @param position The current position of the projectile.
     * @param velocity The current velocity of the projectile.
     * @return The new velocity of the projectile.
     */
    @FunctionalInterface
    public interface ProjectileDynamics {
        Translation3d calculate(double dt, Translation3d position, Translation3d velocity);
    }
}
