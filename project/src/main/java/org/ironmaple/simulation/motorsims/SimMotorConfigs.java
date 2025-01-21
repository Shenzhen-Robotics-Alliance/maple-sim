package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

/**
 *
 *
 * <h1>Stores the configurations of the motor.</h1>
 *
 * <p>This class encapsulates the various configuration parameters required to simulate and control a motor in a system.
 * The configurations include:
 *
 * <ul>
 *   <li><strong>motor:</strong> The motor model used in the simulation (e.g., Falcon 500, NEO).
 *   <li><strong>gearing:</strong> The gear ratio between the motor and the load, affecting the output torque and speed.
 *   <li><strong>loadMOI:</strong> The moment of inertia (MOI) of the load connected to the motor, which determines the
 *       resistance to changes in rotational speed.
 *   <li><strong>friction:</strong> The torque friction characteristics applied to the motor's simulation, representing
 *       real-world losses.
 *   <li><strong>positionVoltageController:</strong> PID controller for controlling the motor's position via voltage.
 *   <li><strong>velocityVoltageController:</strong> PID controller for controlling the motor's velocity via voltage.
 *   <li><strong>positionCurrentController:</strong> PID controller for controlling the motor's position via current.
 *   <li><strong>velocityCurrentController:</strong> PID controller for controlling the motor's velocity via current.
 *   <li><strong>feedforward:</strong> A feedforward controller used to compensate for the desired motor behavior based
 *       on input speeds.
 *   <li><strong>forwardHardwareLimit:</strong> The forward limit for motor rotation, specified in angle units.
 *   <li><strong>reverseHardwareLimit:</strong> The reverse limit for motor rotation, specified in angle units.
 *   <li><strong>currentLimit:</strong> The current limit applied to the motor to protect it from overcurrent
 *       conditions.
 * </ul>
 */
public final class SimMotorConfigs {
    public final DCMotor motor;
    public final double gearing;
    public final MomentOfInertia loadMOI;
    public final Torque friction;

    protected Angle forwardHardwareLimit, reverseHardwareLimit;

    /**
     *
     *
     * <h2>Constructs a simulated motor configuration.</h2>
     *
     * <p>This constructor initializes a {@link SimMotorConfigs} object with the necessary parameters for motor
     * simulation, including the motor model, gearing ratio, load moment of inertia, and friction characteristics.
     *
     * @param motor the motor model to be used in the simulation (e.g., Falcon 500, NEO).
     * @param gearing the gear ratio between the motor and the load, affecting torque and speed output.
     * @param loadMOI the moment of inertia of the load connected to the motor, representing rotational resistance.
     * @param frictionVoltage the voltage applied to simulate frictional torque losses in the motor.
     */
    public SimMotorConfigs(DCMotor motor, double gearing, MomentOfInertia loadMOI, Voltage frictionVoltage) {
        this.motor = motor;
        this.gearing = gearing;
        this.loadMOI = loadMOI;
        this.friction = NewtonMeters.of(motor.getTorque(motor.getCurrent(0, frictionVoltage.in(Volts))));

        forwardHardwareLimit = Radians.of(Double.POSITIVE_INFINITY);
        reverseHardwareLimit = Radians.of(-Double.POSITIVE_INFINITY);
    }

    /**
     *
     *
     * <h2>Calculates the voltage of the motor.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the voltage for a given current and angular velocity.
     *
     * @see DCMotor#getVoltage(double, double) for the underlying implementation.
     * @param current the current flowing through the motor
     * @param mechanismVelocity the final angular velocity of the mechanism
     * @return the voltage required for the motor to achieve the specified current and angular velocity
     */
    public Voltage calculateVoltage(Current current, AngularVelocity mechanismVelocity) {
        return Volts.of(motor.getVoltage(current.in(Amps), mechanismVelocity.in(RadiansPerSecond) * gearing));
    }

    /**
     *
     *
     * <h2>Calculates the velocity of the motor.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the angular velocity for a given current and voltage.
     *
     * @see DCMotor#getSpeed(double, double) for the underlying implementation.
     * @param current the current flowing through the motor.
     * @param voltage the voltage applied to the motor.
     * @return the final angular velocity of the mechanism.
     */
    public AngularVelocity calculateMechanismVelocity(Current current, Voltage voltage) {
        return RadiansPerSecond.of(motor.getSpeed(motor.getTorque(current.in(Amps)), voltage.in(Volts)))
                .div(gearing);
    }

    /**
     *
     *
     * <h2>Calculates the current of the motor.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the current for a given angular velocity and voltage.
     *
     * @see DCMotor#getCurrent(double, double) for the underlying implementation.
     * @param mechanismVelocity the final angular velocity of the mechanism.
     * @param voltage the voltage applied to the moto.
     * @return the current drawn by the motor.
     */
    public Current calculateCurrent(AngularVelocity mechanismVelocity, Voltage voltage) {
        return Amps.of(motor.getCurrent(mechanismVelocity.in(RadiansPerSecond) * gearing, voltage.in(Volts)));
    }

    /**
     *
     *
     * <h2>Calculates the current based on the motor's torque.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the current required for a given torque.
     *
     * @see DCMotor#getCurrent(double) for the underlying implementation.
     * @param torque the final torque generated by the motor on the mechanism.
     * @return the current required to produce the specified torque.
     */
    public Current calculateCurrent(Torque torque) {
        return Amps.of(motor.getCurrent(torque.in(NewtonMeters) / gearing));
    }

    /**
     *
     *
     * <h2>Calculates the torque based on the motor's current.</h2>
     *
     * <p>This method uses the {@link DCMotor} model to find the torque generated by a given current.
     *
     * @see DCMotor#getTorque(double) for the underlying implementation.
     * @param current the current flowing through the motor.
     * @return the torque generated by the motor.
     */
    public Torque calculateTorque(Current current) {
        return NewtonMeters.of(motor.getTorque(current.in(Amps)) * gearing);
    }

    /**
     *
     *
     * <h2>Configures the hard limits for the motor.</h2>
     *
     * <p>This method sets the hardware limits for the motor's movement. When either the forward or reverse limit is
     * reached, the motor will be physically restricted from moving beyond that point, based on the motor's hardware
     * constraints.
     *
     * @param forwardLimit the forward hardware limit angle, beyond which the motor cannot move
     * @param reverseLimit the reverse hardware limit angle, beyond which the motor cannot move
     * @return this instance for method chaining
     */
    public SimMotorConfigs withHardLimits(Angle forwardLimit, Angle reverseLimit) {
        this.forwardHardwareLimit = forwardLimit;
        this.reverseHardwareLimit = reverseLimit;
        return this;
    }

    public AngularVelocity freeSpinMechanismVelocity() {
        return RadiansPerSecond.of(motor.freeSpeedRadPerSec / gearing);
    }

    public Current freeSpinCurrent() {
        return Amps.of(motor.freeCurrentAmps);
    }

    public Current stallCurrent() {
        return Amps.of(motor.stallCurrentAmps);
    }

    public Torque stallTorque() {
        return NewtonMeters.of(motor.stallTorqueNewtonMeters);
    }

    public Voltage nominalVoltage() {
        return Volts.of(motor.nominalVoltageVolts);
    }

    @Override
    protected SimMotorConfigs clone() {
        SimMotorConfigs cfg = new SimMotorConfigs(
                        motor, gearing, loadMOI, Volts.of(motor.getVoltage(friction.in(NewtonMeter), 0.0)))
                .withHardLimits(forwardHardwareLimit, reverseHardwareLimit);

        return cfg;
    }

    @Override
    public String toString() {
        return "SimMotorConfigs {"
                + "\n  motor = " + motor // Relies on DCMotor.toString() for details
                + "\n  gearing = " + gearing
                + "\n  loadMOI (kg·m^2) = " + loadMOI.in(KilogramSquareMeters)
                + "\n  friction (N·m) = " + friction.in(NewtonMeters)
                + "\n}";
    }
}
