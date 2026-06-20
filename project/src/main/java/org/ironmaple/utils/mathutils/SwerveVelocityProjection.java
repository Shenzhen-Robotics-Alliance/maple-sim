// Created by Team 5516 https://github.com/Shenzhen-Robotics-Alliance/ using ChatGPT4o
package org.ironmaple.utils.mathutils;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

public class SwerveVelocityProjection {
    /**
     * Projects the swerve module velocity onto the direction of the current swerve facing.
     *
     * @param swerveVelocity The current velocity and direction of the swerve module.
     * @param currentSwerveFacing The desired direction to project onto.
     * @return The projected velocity in the direction of currentSwerveFacing.
     */
    public static double project(SwerveModuleVelocity swerveVelocity, Rotation2d currentSwerveFacing) {
        // Get the angle of the swerve module's current direction
        Rotation2d swerveModuleAngle = swerveVelocity.angle;

        // Calculate the cosine of the angle difference between swerve module direction and the desired
        // direction
        double cosTheta = Math.cos(swerveModuleAngle.minus(currentSwerveFacing).getRadians());

        // Scale the velocity by the cosine value to get the projection
        return swerveVelocity.velocity * cosTheta;
    }
}
