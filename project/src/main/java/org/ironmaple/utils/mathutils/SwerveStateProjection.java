// Created by Team 5516 https://github.com/Shenzhen-Robotics-Alliance/ using ChatGPT4o
package org.ironmaple.utils.mathutils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveStateProjection {
  /**
   * Projects the swerve module speed onto the direction of the current swerve facing.
   *
   * @param swerveSpeed The current speed and direction of the swerve module.
   * @param currentSwerveFacing The desired direction to project onto.
   * @return The projected speed in the direction of currentSwerveFacing.
   */
  public static double project(SwerveModuleState swerveSpeed, Rotation2d currentSwerveFacing) {
    // Get the angle of the swerve module's current direction
    Rotation2d swerveModuleAngle = swerveSpeed.angle;

    // Calculate the cosine of the angle difference between swerve module direction and the desired
    // direction
    double cosTheta = Math.cos(swerveModuleAngle.minus(currentSwerveFacing).getRadians());

    // Scale the speed by the cosine value to get the projection
    return swerveSpeed.speedMetersPerSecond * cosTheta;
  }
}
