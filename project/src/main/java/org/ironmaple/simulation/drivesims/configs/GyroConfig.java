package org.ironmaple.simulation.drivesims.configs;

public class GyroConfig {
    public double averageDriftingIn30SecsMotionlessDeg;
    public double velocityMeasurementStandardDeviationPercent;

    public GyroConfig(double averageDriftingIn30SecsMotionlessDeg, double velocityMeasurementStandardDeviationPercent) {
        this.averageDriftingIn30SecsMotionlessDeg = averageDriftingIn30SecsMotionlessDeg;
        this.velocityMeasurementStandardDeviationPercent = velocityMeasurementStandardDeviationPercent;
    }

    public GyroConfig() {
    }

    public GyroConfig withAverageDriftingIn30SecsMotionlessDeg(double averageDriftingIn30SecsMotionlessDeg) {
        this.averageDriftingIn30SecsMotionlessDeg = averageDriftingIn30SecsMotionlessDeg;
        return this;
    }

    public GyroConfig withVelocityMeasurementStandardDeviationPercent(double velocityMeasurementStandardDeviationPercent) {
        this.velocityMeasurementStandardDeviationPercent = velocityMeasurementStandardDeviationPercent;
        return this;
    }

    /**
     *
     *
     * <h2>Creates the Simulation for a <a href="https://store.ctr-electronics.com/pigeon-2/">CTRE Pigeon 2 IMU</a>.
     * </h2>
     *
     * @return a gyro simulation factory configured for the Pigeon 2 IMU
     */
    public static GyroConfig createPigeon2() {
        /*
         * user manual of pigeon 2:
         * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
         * */
        return new GyroConfig(0.5, 0.02);
    }

    /**
     *
     *
     * <h2>Creates the Simulation for a <a href="https://pdocs.kauailabs.com/navx-mxp/">navX2-MXP IMU</a>.</h2>
     *
     * @return a gyro simulation factory configured for the navX2-MXP IMU
     */
    public static GyroConfig getNav2X() {
        return new GyroConfig(2, 0.04);
    }
}
