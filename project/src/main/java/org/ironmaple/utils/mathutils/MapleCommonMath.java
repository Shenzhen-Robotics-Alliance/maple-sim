package org.ironmaple.utils.mathutils;

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

  public static double constrainMagnitude(double value, double maxMagnitude) {
    return Math.copySign(Math.min(Math.abs(value), Math.abs(maxMagnitude)), value);
  }
}
