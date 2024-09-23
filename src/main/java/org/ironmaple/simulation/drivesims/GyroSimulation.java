package org.ironmaple.simulation.drivesims;


import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.utils.mathutils.MapleCommonMath;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import static org.ironmaple.simulation.SimulatedArena.*;

/**
 * Simulation for a IMU module used as gyro.
 *
 * The Simulation is basically an indefinite integral of the angular velocity during each simulation sub ticks.
 * Above that, it also musicales the measurement inaccuracy of the gyro, drifting in no-motion and drifting due to impacts.
 * */
public class GyroSimulation {
    private static final double ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING = Math.toRadians(60) / 0.05, MAX_DRIFT_IN_1_TICK = Math.toRadians(20);
    private final double AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS_DEG, VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT;

    private Rotation2d gyroReading;
    private double measuredAngularVelocityRadPerSec, previousAngularVelocityRadPerSec;
    private Queue<Rotation2d> cachedRotations;

    /**
     * creates a gyro simulation
     * @param AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS_DEG the average amount of drifting of the gyro, in degrees, if it was on a vibrating platform but staying still in rotation for 30 seconds, you can often find this value in the user manual
     * @param VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT the standard deviation of the velocity measurement, usually around 0.05
     * */
    public GyroSimulation(double AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS_DEG, double VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT) {
        this.AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS_DEG = AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS_DEG;
        this.VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT = VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT;

        gyroReading = new Rotation2d();
        this.previousAngularVelocityRadPerSec = this.measuredAngularVelocityRadPerSec = 0;
        this.cachedRotations = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++)
            cachedRotations.offer(gyroReading);
    }

    /**
     * <p>calibrates the rotation of the gyro to a given rotation,
     * similar to Pigeon2().setYaw().</p>
     * <p>The gyro will continue to estimate the rotation using indefinite integral of velocity, added to this rotation.</p>
     * @param currentRotation the current rotation of the robot
     * */
    public void setRotation(Rotation2d currentRotation) {
        this.gyroReading = currentRotation;
    }

    /**
     * <p>@return the current reading of the gyro</p>
     * <p>NOTE: the rotation increases continuously, it does not roll over every 360 degrees</p>
     * */
    public Rotation2d getGyroReading() {
        return gyroReading;
    }

    /**
     * gyro readings for <a href="https://v6.docs.ctr-electronics.com/en/stable/docs/application-notes/update-frequency-impact.html">high-frequency odometers</a>.
     * @return the readings of the gyro during the last 5 simulation sub ticks
     * */
    public Rotation2d[] getCachedGyroReadings() {
        return cachedRotations.toArray(Rotation2d[]::new);
    }

    /**
     * updates the gyro simulation
     * this method should be called in every sub-tick of the simulation
     * if you want to use this class out of {@link org.ironmaple.simulation.SimulatedArena}, please call it 5 times in a roll.
     * */
    public void updateSimulationSubTick(double actualAngularVelocityRadPerSec) {
        final Rotation2d driftingDueToImpact = getDriftingDueToImpact(actualAngularVelocityRadPerSec);
        gyroReading = gyroReading.plus(driftingDueToImpact);

        final Rotation2d dTheta = getGyroDTheta(actualAngularVelocityRadPerSec);
        gyroReading = gyroReading.plus(dTheta);

        final Rotation2d noMotionDrifting = getNoMotionDrifting();
        gyroReading = gyroReading.plus(noMotionDrifting);

        cachedRotations.poll(); cachedRotations.offer(gyroReading);
    }

    private Rotation2d getDriftingDueToImpact(double actualAngularVelocityRadPerSec) {
        final double angularAccelerationRadPerSecSq = (actualAngularVelocityRadPerSec - previousAngularVelocityRadPerSec) / SIMULATION_DT,
                driftingDueToImpactDegUnlimitedAbsVal = Math.abs(angularAccelerationRadPerSecSq) > ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING ?
                        Math.abs(angularAccelerationRadPerSecSq) / ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING * Math.toRadians(5) : 0,
                driftingDueToImpactDegAbsVal = Math.min(driftingDueToImpactDegUnlimitedAbsVal, MAX_DRIFT_IN_1_TICK),
                driftingDueToImpactDeg = Math.copySign(driftingDueToImpactDegAbsVal, -angularAccelerationRadPerSecSq);

        previousAngularVelocityRadPerSec = actualAngularVelocityRadPerSec;

        return Rotation2d.fromDegrees(driftingDueToImpactDeg);
    }

    private Rotation2d getGyroDTheta(double actualAngularVelocityRadPerSec) {
        this.measuredAngularVelocityRadPerSec = actualAngularVelocityRadPerSec *
                MapleCommonMath.generateRandomNormal(1, VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT);
        return Rotation2d.fromRadians(actualAngularVelocityRadPerSec * SIMULATION_DT);
    }

    private Rotation2d getNoMotionDrifting() {
        final double AVERAGE_DRIFTING_1_PERIOD = this.AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS_DEG
                / 30 * SIMULATION_DT,
                driftingInThisPeriod = MapleCommonMath.generateRandomNormal(0, AVERAGE_DRIFTING_1_PERIOD);

        return Rotation2d.fromDegrees(driftingInThisPeriod);
    }

    /**
     * creates the simulation for a <a href = "https://store.ctr-electronics.com/pigeon-2/">CTRE Pigeon 2 IMU</a>
     * */
    public static GyroSimulation createPigeon2() {
        /*
         * according to the user manual of pigeon 2, which can be found here:
         * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
         *
         * The gyro drifts 0.1 degrees every 30 seconds when motionless (as they claim)
         * */

        return new GyroSimulation(
                0.5,
                5e-3
        );
    }

    /**
     * creates the simulation for a <a href = "https://pdocs.kauailabs.com/navx-mxp/">navX2-MXP IMU</a>
     * */
    public static GyroSimulation createNav2X() {
        return new GyroSimulation(
                2,
                0.01
        );
    }

    /**
     * creates the simulation for a generic, low-accuracy imu, such as the one integrated into the RobotRIO
     * */
    public static GyroSimulation createGeneric() {
        return new GyroSimulation(
                5,
                0.05
        );
    }
}
