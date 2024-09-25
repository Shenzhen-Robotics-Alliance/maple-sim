package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import static org.ironmaple.simulation.SimulatedArena.SIMULATION_DT;
import static org.ironmaple.simulation.SimulatedArena.SIMULATION_SUB_TICKS_IN_1_PERIOD;

public class SwerveModuleSimulation {
    private final DCMotor DRIVE_MOTOR;
    private final DCMotorSim steerMotorSim;
    private final double
            DRIVE_CURRENT_LIMIT,
            DRIVE_GEAR_RATIO, STEER_GEAR_RATIO,
            DRIVE_FRICTION_VOLTAGE, STEER_FRICTION_VOLTAGE,
            TIRE_COEFFICIENT_OF_FRICTION, WHEEL_RADIUS_METERS,
            DRIVE_WHEEL_INERTIA = 0.01;

    private double driveMotorRequestedVolts = 0.0, steerMotorAppliedVolts = 0.0, driveMotorAppliedVolts = 0.0,
            driveMotorSupplyCurrentAmps = 0.0, steerMotorSupplyCurrentAmps = 0.0,
            steerRelativeEncoderPositionRad = 0.0, steerRelativeEncoderSpeedRadPerSec = 0.0,
            steerAbsoluteEncoderSpeedRadPerSec = 0.0,
            driveEncoderUnGearedPositionRad = 0.0, driveEncoderUnGearedSpeedRadPerSec = 0.0;
    private Rotation2d steerAbsoluteFacing = Rotation2d.fromRotations(Math.random());

    private final double steerRelativeEncoderOffSet = (Math.random() - 0.5) * 30;

    private final Queue<Double> cachedSteerRelativeEncoderPositionsRad, cachedDriveEncoderUnGearedPositionsRad;
    private final Queue<Rotation2d> cachedSteerAbsolutePositions;
    public SwerveModuleSimulation(DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimit, double driveGearRatio, double steerGearRatio, double driveFrictionVoltage, double steerFrictionVoltage, double tireCoefficientOfFriction, double wheelsRadiusMeters, double steerRotationalInertia) {
        DRIVE_MOTOR = driveMotor;
        DRIVE_CURRENT_LIMIT = driveCurrentLimit;
        DRIVE_GEAR_RATIO = driveGearRatio;
        STEER_GEAR_RATIO = steerGearRatio;
        DRIVE_FRICTION_VOLTAGE = driveFrictionVoltage;
        STEER_FRICTION_VOLTAGE = steerFrictionVoltage;
        TIRE_COEFFICIENT_OF_FRICTION = tireCoefficientOfFriction;
        WHEEL_RADIUS_METERS = wheelsRadiusMeters;

        this.steerMotorSim = new DCMotorSim(steerMotor, STEER_GEAR_RATIO, steerRotationalInertia);

        this.cachedDriveEncoderUnGearedPositionsRad = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++)
            cachedDriveEncoderUnGearedPositionsRad.offer(driveEncoderUnGearedPositionRad);
        this.cachedSteerRelativeEncoderPositionsRad = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++)
            cachedSteerRelativeEncoderPositionsRad.offer(steerRelativeEncoderPositionRad);
        this.cachedSteerAbsolutePositions = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++)
            cachedSteerAbsolutePositions.offer(steerAbsoluteFacing);

        this.steerRelativeEncoderPositionRad = steerAbsoluteFacing.getRadians() * STEER_GEAR_RATIO + steerRelativeEncoderOffSet;
    }

    public void requestDriveVoltageOut(double volts) {
        this.driveMotorRequestedVolts = volts;
    }

    public void requestSteerVoltageOut(double volts) {
        this.steerMotorAppliedVolts = volts;
        this.steerMotorSim.setInputVoltage(MathUtil.applyDeadband(
                volts,
                STEER_FRICTION_VOLTAGE,
                12
        ));
    }

    public double getDriveMotorAppliedVolts() {
        return driveMotorAppliedVolts;
    }

    public double getSteerMotorAppliedVolts() {
        return steerMotorAppliedVolts;
    }

    public double getDriveMotorSupplyCurrentAmps() {
        return driveMotorSupplyCurrentAmps;
    }

    public double getSteerMotorSupplyCurrentAmps() {
        return steerMotorSupplyCurrentAmps;
    }

    public double getDriveEncoderUnGearedPositionRad() {
        return driveEncoderUnGearedPositionRad;
    }
    public double getDriveEncoderFinalPositionRad() {
        return getDriveEncoderUnGearedPositionRad() / DRIVE_GEAR_RATIO;
    }
    public double getDriveEncoderUnGearedSpeedRadPerSec() {
        return driveEncoderUnGearedSpeedRadPerSec;
    }
    public double getDriveEncoderFinalSpeedRadPerSec() {
        return getDriveEncoderUnGearedSpeedRadPerSec() / DRIVE_GEAR_RATIO;
    }

    public double getSteerRelativeEncoderPositionRad() {
        return steerRelativeEncoderPositionRad;
    }
    public double getSteerRelativeEncoderSpeedRadPerSec() {
        return steerRelativeEncoderSpeedRadPerSec;
    }

    public Rotation2d getSteerAbsoluteFacing() {
        return steerAbsoluteFacing;
    }
    public double getSteerAbsoluteEncoderSpeedRadPerSec() {
        return steerAbsoluteEncoderSpeedRadPerSec;
    }

    public double[] getCachedDriveEncoderUnGearedPositionsRad() {
        return cachedDriveEncoderUnGearedPositionsRad.stream().mapToDouble(value -> value).toArray();
    }
    public double[] getCachedDriveWheelFinalPositionsRad() {
        return cachedDriveEncoderUnGearedPositionsRad.stream().mapToDouble(value -> value / DRIVE_GEAR_RATIO).toArray();
    }


    public double[] getCachedSteerRelativeEncoderPositions() {
        return cachedSteerRelativeEncoderPositionsRad.stream().mapToDouble(value -> value).toArray();
    }

    public Rotation2d[] getCachedSteerAbsolutePositions() {
        return cachedSteerAbsolutePositions.toArray(Rotation2d[]::new);
    }

    /**
     * updates the simulation sub-tick for this module, updating its inner status (sensor readings) and calculating a total force
     * @param moduleCurrentGroundVelocity
     * @return
     * */
    public Force updateSimulationSubTick(Vector2 moduleCurrentGroundVelocity, Rotation2d robotFacing, double gravityForceOnModuleNewtons) {
        updateSteerSimulation();

        /* the maximum gripping force that the wheel can generate */
        final double grippingForceNewtons = gravityForceOnModuleNewtons * TIRE_COEFFICIENT_OF_FRICTION;
        final Rotation2d moduleWorldFacing = this.steerAbsoluteFacing.plus(robotFacing);
        final Vector2 propellingForce = getPropellingForce(grippingForceNewtons, moduleWorldFacing, moduleCurrentGroundVelocity);
        updateEncoderTicks();

        /* simulate the friction force */
        final Vector2
                frictionalForce = getFrictionalForce(moduleWorldFacing, moduleCurrentGroundVelocity, grippingForceNewtons),
                totalForceUnconstrained = propellingForce.copy().add(frictionalForce),
                totalForce = Vector2.create(
                        Math.min(totalForceUnconstrained.getMagnitude(), grippingForceNewtons),
                        totalForceUnconstrained.getDirection()
                );

        return new Force(totalForce);
    }

    /**
     *
     * */
    private void updateSteerSimulation() {
        steerMotorSim.update(SIMULATION_DT);

        /* update the readings of the sensor */
        this.steerAbsoluteFacing = Rotation2d.fromRadians(steerMotorSim.getAngularPositionRad());
        this.steerRelativeEncoderPositionRad = steerAbsoluteFacing.getRadians() * STEER_GEAR_RATIO + steerRelativeEncoderOffSet;
        this.steerAbsoluteEncoderSpeedRadPerSec = steerMotorSim.getAngularVelocityRadPerSec();
        this.steerRelativeEncoderSpeedRadPerSec = steerAbsoluteEncoderSpeedRadPerSec * STEER_GEAR_RATIO;

        /* cache sensor readings to queue for high-frequency odometry */
        this.cachedSteerAbsolutePositions.poll(); this.cachedSteerAbsolutePositions.offer(steerAbsoluteFacing);
        this.cachedSteerRelativeEncoderPositionsRad.poll(); this.cachedSteerRelativeEncoderPositionsRad.offer(steerRelativeEncoderPositionRad);
    }

    private Vector2 getPropellingForce(double grippingForceNewtons, Rotation2d moduleWorldFacing, Vector2 moduleCurrentGroundVelocity) {
        final double theoreticalMaxPropellingForceNewtons = getDriveWheelTorque() / WHEEL_RADIUS_METERS;
        final boolean skidding = Math.abs(theoreticalMaxPropellingForceNewtons) > grippingForceNewtons;
        final double propellingForceNewtons;
        if (skidding)
            propellingForceNewtons = Math.copySign(grippingForceNewtons, theoreticalMaxPropellingForceNewtons);
        else
            propellingForceNewtons = theoreticalMaxPropellingForceNewtons;

        final double floorVelocityProjectionOnWheelDirectionMPS = moduleCurrentGroundVelocity.getMagnitude() *
                Math.cos(moduleCurrentGroundVelocity.getAngleBetween(new Vector2(moduleWorldFacing.getRadians())));

        if (skidding) // if the chassis is skidding, part of the toque will cause the wheels to spin freely
            this.driveEncoderUnGearedSpeedRadPerSec += getDriveWheelTorque() / DRIVE_WHEEL_INERTIA * SIMULATION_DT * 0.3;
        else  // if the chassis is tightly gripped on floor, the floor velocity is projected to the wheel
            this.driveEncoderUnGearedSpeedRadPerSec = floorVelocityProjectionOnWheelDirectionMPS / WHEEL_RADIUS_METERS * DRIVE_GEAR_RATIO;

        return Vector2.create(propellingForceNewtons, moduleWorldFacing.getRadians());
    }

    private double getDriveWheelTorque() {
        final double currentAtRequestedVolts = DRIVE_MOTOR.getCurrent(
                this.driveEncoderUnGearedSpeedRadPerSec,
                driveMotorRequestedVolts
        );

        driveMotorAppliedVolts = driveMotorRequestedVolts;
        /* normally, motor controller starts cutting the supply voltage when the current exceed 150% the current limit */
        final boolean currentTooHigh = Math.abs(currentAtRequestedVolts) > 1.2 * DRIVE_CURRENT_LIMIT,
                driveMotorTryingToAccelerate = driveMotorRequestedVolts * driveMotorSupplyCurrentAmps > 0;

        if (currentTooHigh && driveMotorTryingToAccelerate) {
            /* activate current limit, cut down the applied voltage to match current limit */
            final double currentWithLimits = Math.copySign(DRIVE_CURRENT_LIMIT, currentAtRequestedVolts);
            driveMotorAppliedVolts = DRIVE_MOTOR.getVoltage(
                    DRIVE_MOTOR.getTorque(currentWithLimits),
                    this.driveEncoderUnGearedSpeedRadPerSec
            );
        }

        driveMotorAppliedVolts = MathUtil.clamp(
                driveMotorAppliedVolts,
                -12, 12
        );

        /* calculate the actual supply current */
        driveMotorSupplyCurrentAmps = DRIVE_MOTOR.getCurrent(
                this.driveEncoderUnGearedSpeedRadPerSec,
                MathUtil.applyDeadband(
                        driveMotorAppliedVolts,
                        DRIVE_FRICTION_VOLTAGE,
                        12
                )
        );

        /* calculate the torque generated,  */
        final double torqueOnRotter = DRIVE_MOTOR.getTorque(driveMotorSupplyCurrentAmps);
        return torqueOnRotter * DRIVE_GEAR_RATIO;
    }

    private void updateEncoderTicks() {
        this.driveEncoderUnGearedSpeedRadPerSec += this.driveEncoderUnGearedSpeedRadPerSec * SIMULATION_DT;
        this.cachedDriveEncoderUnGearedPositionsRad.poll(); this.cachedDriveEncoderUnGearedPositionsRad.offer(driveEncoderUnGearedPositionRad);
    }

    private Vector2 getFrictionalForce(Rotation2d moduleWorldFacing, Vector2 moduleCurrentGroundVelocity, double grippingForceNewtons) {
        final Vector2 moduleCurrentFreeVelocity = Vector2.create(
                getDriveEncoderFinalSpeedRadPerSec() * WHEEL_RADIUS_METERS,
                moduleWorldFacing.getRadians()
        );
        final Vector2 differenceBetweenGroundAndFreeVelocity = moduleCurrentFreeVelocity.difference(moduleCurrentGroundVelocity);
        final double FRICTIONAL_FORCE_GAIN = 3.0;
        final Vector2
                frictionalForceUnconstrained = differenceBetweenGroundAndFreeVelocity.copy().multiply(FRICTIONAL_FORCE_GAIN);

        return Vector2.create(
                Math.min(frictionalForceUnconstrained.getMagnitude(), grippingForceNewtons),
                frictionalForceUnconstrained.getDirection()
        );
    }


    public enum DRIVE_WHEEL_TYPE {
        RUBBER,
        TIRE
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS Mark4 Swerve Module</a> for simulation
     * */
    public static SwerveModuleSimulation createMark4(DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, DRIVE_WHEEL_TYPE driveWheelType, int gearRatioLevel) {
        return new SwerveModuleSimulation(
                driveMotor, steerMotor, driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.14;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                12.8,
                0.2,
                0.2,
                switch (driveWheelType) {
                    case RUBBER -> 0.95;
                    case TIRE -> 1.05;
                },
                Units.inchesToMeters(2),
                0.05
        );
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module">SDS Mark4-i Swerve Module</a> for simulation
     * */
    public static SwerveModuleSimulation createMark4i(DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, DRIVE_WHEEL_TYPE driveWheelType, int gearRatioLevel) {
        return new SwerveModuleSimulation(
                driveMotor, steerMotor, driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.15;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                150.0/7.0,
                0.2,
                0.2,
                switch (driveWheelType) {
                    case RUBBER -> 0.95;
                    case TIRE -> 1.05;
                },
                Units.inchesToMeters(2),
                0.05
        );
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/products/mk4n-swerve-module">SDS Mark4-n Swerve Module</a> for simulation
     * */
    public static SwerveModuleSimulation createMark4n(DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, DRIVE_WHEEL_TYPE driveWheelType, int gearRatioLevel) {
        return new SwerveModuleSimulation(
                driveMotor, steerMotor, driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 7.13;
                    case 2 -> 5.9;
                    case 3 -> 5.36;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                18.75,
                0.25,
                0.25,
                switch (driveWheelType) {
                    case RUBBER -> 0.95;
                    case TIRE -> 1.05;
                },
                Units.inchesToMeters(2),
                0.05
        );
    }
}
