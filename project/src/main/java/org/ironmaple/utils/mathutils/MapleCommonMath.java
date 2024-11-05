package org.ironmaple.utils.mathutils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Random;

public class MapleCommonMath {
    private static final Random random = new Random();

    /**
     * using the random number generator of a fixed seed, generate the next random normal variable
     *
     * @param mean the center of the distribution
     * @param stdDev the standard deviation of the distribution
     * @return the next random variable x from the distribution
     */
    public static double generateRandomNormal(double mean, double stdDev) {
        double u1 = random.nextDouble();
        double u2 = random.nextDouble();
        // Box–Muller transform https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
        double z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        return z0 * stdDev + mean;
    }

    /**
     *
     *
     * <h2>Constrains the magnitude of a value.</h2>
     *
     * @param value the value to constrain
     * @param maxMagnitude the maximum allowed magnitude
     * @return the constrained value
     */
    public static double constrainMagnitude(double value, double maxMagnitude) {
        return Math.copySign(Math.min(Math.abs(value), Math.abs(maxMagnitude)), value);
    }

    public static double linearInterpretationWithBounding(double x1, double y1, double x2, double y2, double x) {
        final double minX = Math.min(x1, x2), maxX = Math.max(x1, x2);
        return linearInterpretation(x1, y1, x2, y2, Math.min(maxX, Math.max(minX, x)));
    }

    public static double linearInterpretation(double x1, double y1, double x2, double y2, double x) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    /**
     *
     *
     * <h2>Obtains the angle of a translation, ignores zero translation.</h2>
     *
     * <p>This prevents the driver-station from throwing a bunch of "x and y components of Rotation2d are zero" warning.
     *
     * @param translation2d the translation
     * @return the angle of the translation, or zero if the translation is zero
     */
    public static Rotation2d getAngle(Translation2d translation2d) {
        return translation2d.getNorm() == 0 ? Rotation2d.fromDegrees(0) : translation2d.getAngle();
    }
}
