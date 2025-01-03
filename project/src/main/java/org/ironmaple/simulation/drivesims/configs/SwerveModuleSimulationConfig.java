package org.ironmaple.simulation.drivesims.configs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

public class SwerveModuleSimulationConfig implements Supplier<SwerveModuleSimulation> {
    public final SimMotorConfigs driveMotorConfigs, steerMotorConfigs;
    public final double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEELS_COEFFICIENT_OF_FRICTION;
    public final Voltage DRIVE_FRICTION_VOLTAGE;
    public final Distance WHEEL_RADIUS;

    /**
     *
     *
     * <h2>Constructs a Configuration for Swerve Module Simulation.</h2>
     *
     * <p>If you are using {@link SimulatedArena#overrideSimulationTimings(Time, int)} to use custom timings, you must
     * call the method before constructing any swerve module simulations using this constructor.
     *
     * @param driveMotorModel the model of the driving motor
     * @param steerMotorModel; the model of the steering motor
     * @param driveGearRatio the gear ratio for the driving motor, >1 is reduction
     * @param steerGearRatio the gear ratio for the steering motor, >1 is reduction
     * @param driveFrictionVoltage the measured minimum amount of voltage that can turn the driving rotter
     * @param steerFrictionVoltage the measured minimum amount of voltage that can turn the steering rotter
     * @param wheelRadius the radius of the wheels.
     * @param steerRotationalInertia the rotational inertia of the entire steering mechanism
     * @param wheelsCoefficientOfFriction the <a
     *     href='https://simple.wikipedia.org/wiki/Coefficient_of_friction#:~:text=A%20coefficient%20of%20friction%20is%20a%20value%20that%20shows%20the'>coefficient
     *     of friction</a> of the tires, normally around 1.2 {@link Units#inchesToMeters(double)}.
     */
    public SwerveModuleSimulationConfig(
            DCMotor driveMotorModel,
            DCMotor steerMotorModel,
            double driveGearRatio,
            double steerGearRatio,
            Voltage driveFrictionVoltage,
            Voltage steerFrictionVoltage,
            Distance wheelRadius,
            MomentOfInertia steerRotationalInertia,
            double wheelsCoefficientOfFriction) {
        BoundingCheck.check(driveGearRatio, 4, 18, "drive gear ratio", "times reduction");
        BoundingCheck.check(steerGearRatio, 10, 50, "steer gear ratio", "times reduction");
        BoundingCheck.check(driveFrictionVoltage.in(Volts), 0, 0.35, "drive friction voltage", "volts");
        BoundingCheck.check(steerFrictionVoltage.in(Volts), 0, 0.6, "steer friction voltage", "volts");
        BoundingCheck.check(wheelRadius.in(Inches), 1, 3.2, "drive wheel radius", "inches");
        BoundingCheck.check(
                steerRotationalInertia.in(KilogramSquareMeters), 0.005, 0.05, "steer rotation inertia", "kg * m^2");
        BoundingCheck.check(wheelsCoefficientOfFriction, 0.6, 2, "tire coefficient of friction", "");

        this.driveMotorConfigs =
                new SimMotorConfigs(driveMotorModel, driveGearRatio, KilogramSquareMeters.zero(), driveFrictionVoltage);
        this.steerMotorConfigs =
                new SimMotorConfigs(steerMotorModel, steerGearRatio, steerRotationalInertia, steerFrictionVoltage);
        DRIVE_GEAR_RATIO = driveGearRatio;
        STEER_GEAR_RATIO = steerGearRatio;
        WHEELS_COEFFICIENT_OF_FRICTION = wheelsCoefficientOfFriction;
        DRIVE_FRICTION_VOLTAGE = driveFrictionVoltage;
        WHEEL_RADIUS = wheelRadius;
    }

    @Override
    public SwerveModuleSimulation get() {
        return new SwerveModuleSimulation(this);
    }

    public double getGrippingForceNewtons(double gravityForceOnModuleNewtons) {
        return gravityForceOnModuleNewtons * WHEELS_COEFFICIENT_OF_FRICTION;
    }

    /**
     *
     *
     * <h2>Obtains the theoretical speed that the module can achieve.</h2>
     *
     * @return the theoretical maximum ground speed that the module can achieve, in m/s
     */
    public LinearVelocity maximumGroundSpeed() {
        return MetersPerSecond.of(
                driveMotorConfigs.freeSpinMechanismVelocity().in(RadiansPerSecond) * WHEEL_RADIUS.in(Meters));
    }

    /**
     *
     *
     * <h2>Obtains the theoretical maximum propelling force of ONE module.</h2>
     *
     * <p>Calculates the maximum propelling force with respect to the gripping force and the drive motor's torque under
     * its current limit.
     *
     * @param robotMass the mass of the robot
     * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
     * @return the maximum propelling force of EACH module
     */
    public Force getTheoreticalPropellingForcePerModule(Mass robotMass, int modulesCount, Current statorCurrentLimit) {
        final double
                maxThrustNewtons =
                        driveMotorConfigs.calculateTorque(statorCurrentLimit).in(NewtonMeters)
                                / WHEEL_RADIUS.in(Meters),
                maxGrippingNewtons = 9.8 * robotMass.in(Kilograms) / modulesCount * WHEELS_COEFFICIENT_OF_FRICTION;

        return Newtons.of(Math.min(maxThrustNewtons, maxGrippingNewtons));
    }

    /**
     *
     *
     * <h2>Obtains the theatrical linear acceleration that the robot can achieve.</h2>
     *
     * <p>Calculates the maximum linear acceleration of a robot, with respect to its mass and
     * {@link #getTheoreticalPropellingForcePerModule(Mass, int, Current)}.
     *
     * @param robotMass the mass of the robot
     * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
     */
    public LinearAcceleration maxAcceleration(Mass robotMass, int modulesCount, Current statorCurrentLimit) {
        return getTheoreticalPropellingForcePerModule(robotMass, modulesCount, statorCurrentLimit)
                .times(modulesCount)
                .div(robotMass);
    }
}
