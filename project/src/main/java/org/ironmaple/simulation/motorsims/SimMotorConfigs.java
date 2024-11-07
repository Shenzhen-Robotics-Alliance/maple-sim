package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public final class SimMotorConfigs {
    public final DCMotor motor;
    public final double gearing;
    public final MomentOfInertia loadMOI;
    public final Torque friction;

    /* all these controllers receive un-geared position/velocity from the encoder, not the final position/velocity of the mechanism */
    public final PIDController positionVoltageController;
    public final PIDController velocityVoltageController;
    public final PIDController positionCurrentController;
    public final PIDController velocityCurrentController;
    public SimpleMotorFeedforward feedforward;

    protected Angle forwardHardwareLimit;
    protected Angle reverseHardwareLimit;
    protected Current currentLimit;

    public SimMotorConfigs(DCMotor motor, double gearing, MomentOfInertia loadMOI, Voltage frictionVoltage) {
        this.motor = motor;
        this.gearing = gearing;
        this.loadMOI = loadMOI;
        this.friction = NewtonMeters.of(motor.getTorque(motor.getCurrent(0, frictionVoltage.in(Volts))));
        this.positionVoltageController = new PIDController(0, 0, 0);
        this.positionCurrentController = new PIDController(0, 0, 0);
        this.velocityVoltageController = new PIDController(0, 0, 0);
        this.velocityCurrentController = new PIDController(0, 0, 0);
        this.forwardHardwareLimit = Radians.of(Double.POSITIVE_INFINITY);
        this.reverseHardwareLimit = Radians.of(Double.NEGATIVE_INFINITY);
        this.currentLimit = Amps.of(150);

        this.withFeedForward(
                frictionVoltage,
                VoltsPerRadianPerSecond.ofNative(motor.nominalVoltageVolts / motor.freeSpeedRadPerSec),
                VoltsPerRadianPerSecondSquared.ofNative(0),
                Seconds.of(0.02));
    }

    public Voltage calculateVoltage(Current current, AngularVelocity angularVelocity) {
        // find what voltage is needed to get the current
        return Volts.of(motor.getVoltage(current.in(Amps), angularVelocity.in(RadiansPerSecond) * gearing));
    }

    public AngularVelocity calculateVelocity(Current current, Voltage voltage) {
        return RadiansPerSecond.of(motor.getSpeed(motor.getTorque(current.in(Amps)), voltage.in(Volts)));
    }

    public Current calculateCurrent(AngularVelocity angularVelocity, Voltage voltage) {
        return Amps.of(motor.getCurrent(angularVelocity.in(RadiansPerSecond), voltage.in(Volts)));
    }

    public Torque calculateTorque(Current current) {
        return NewtonMeters.of(motor.getTorque(current.in(Amps)));
    }

    public SimMotorConfigs withFeedForward(
            Voltage kS,
            Per<VoltageUnit, AngularVelocityUnit> kV,
            Per<VoltageUnit, AngularAccelerationUnit> kA,
            Time dt) {
        var kVUnit = PerUnit.combine(Volts, RadiansPerSecond);
        var kAUnit = PerUnit.combine(Volts, RadiansPerSecondPerSecond);
        this.feedforward = new SimpleMotorFeedforward(kS.in(Volts), kV.in(kVUnit), kA.in(kAUnit), dt.in(Seconds));
        return this;
    }

    /**
     * Configures the PD controller for Positional Requests using
     * {@link org.ironmaple.simulation.motorsims.requests.PositionVoltage} request.
     *
     * <p>This is unit safe and can be configure like so:
     *
     * <pre><code>
     * // Volts per Rotation of error is how CTRE handles PID when used with voltage requests
     * sim.withPositionalVoltageController(
     *   Volts.per(Rotation).ofNative(100.0),
     *   Volts.per(RotationsPerSecond).ofNative(5.0)
     * );
     * </code></pre>
     *
     * @param kP the proportional gain
     * @param kD the derivative gain
     * @return this instance for method chaining
     */
    public SimMotorConfigs withPositionalVoltageController(
            Per<VoltageUnit, AngleUnit> kP, Per<VoltageUnit, AngularVelocityUnit> kD) {
        var kPUnit = PerUnit.combine(Volts, Radians);
        var kDUnit = PerUnit.combine(Volts, RadiansPerSecond);
        positionVoltageController.setP(kP.in(kPUnit));
        positionVoltageController.setD(kD.in(kDUnit));
        return this;
    }

    /**
     * Configures the PD controller for {@link org.ironmaple.simulation.motorsims.requests.VelocityVoltage} requests.
     *
     * <p>This is unit safe and can be configure like so:
     *
     * <pre><code>
     * // Volts per RPS of error is how CTRE handles PID when used with voltage requests
     * sim.withVelocityVoltageController(
     *   Volts.per(RotationsPerSecond).ofNative(0.4)
     * );
     * </code></pre>
     *
     * @param kP the proportional gain
     * @return this instance for method chaining
     */
    public SimMotorConfigs withVelocityVoltageController(Per<VoltageUnit, AngularVelocityUnit> kP) {
        var kPUnit = PerUnit.combine(Volts, RadiansPerSecond);
        velocityVoltageController.setP(kP.in(kPUnit));
        return this;
    }

    /**
     * Configures the PD controller for Positional Requests using
     * {@link org.ironmaple.simulation.motorsims.requests.PositionVoltage} requests.
     *
     * <p>This is unit safe and can be configure like so:
     *
     * <pre><code>
     * // Amps per Rotation of error is how CTRE handles PID when used with current requests
     * sim.withPositionalCurrentController(
     *   Amps.per(Rotation).ofNative(100.0),
     *   Amps.per(RotationsPerSecond).ofNative(5.0)
     * );
     * </code></pre>
     *
     * @param kP the proportional gain
     * @param kD the derivative gain
     * @return this instance for method chaining
     */
    public SimMotorConfigs withPositionalCurrentController(
            Per<CurrentUnit, AngleUnit> kP, Per<CurrentUnit, AngularVelocityUnit> kD) {
        var kPUnit = PerUnit.combine(Amps, Radians);
        var kDUnit = PerUnit.combine(Amps, RadiansPerSecond);
        positionCurrentController.setP(kP.in(kPUnit));
        positionCurrentController.setD(kD.in(kDUnit));
        return this;
    }

    /**
     * Configures the PD controller for Velocity Requests using
     *
     * <p>This is unit safe and can be configure like so:
     *
     * <pre><code>
     * // Amps per RPS of error is how CTRE handles PID when used with current requests
     * sim.withVelocityCurrentController(
     *   Amps.per(RotationsPerSecond).ofNative(0.4)
     * );
     * </code></pre>
     *
     * @param kP the proportional gain
     * @return this instance for method chaining
     */
    public SimMotorConfigs withVelocityCurrentController(Per<CurrentUnit, AngularVelocityUnit> kP) {
        var kPUnit = PerUnit.combine(Amps, RadiansPerSecond);
        velocityCurrentController.setP(kP.in(kPUnit));
        return this;
    }

    /**
     * Configures the positionaly controllers to use continuous wrap.
     *
     * @see PIDController#enableContinuousInput(double, double)
     */
    public SimMotorConfigs withControllerContinousInput() {
        return this.withControllerContinousInput(Rotations.of(0), Rotations.of(1));
    }

    /**
     * Configures the positionaly controllers to use continuous wrap.
     *
     * @param min the minimum angle
     * @param max the maximum angle
     * @return this instance for method chaining
     * @see PIDController#enableContinuousInput(double, double)
     */
    public SimMotorConfigs withControllerContinousInput(Angle min, Angle max) {
        positionVoltageController.enableContinuousInput(min.in(Radians), max.in(Radians));
        positionCurrentController.enableContinuousInput(min.in(Radians), max.in(Radians));
        return this;
    }

    /**
     * Configures the current limit for the motor.
     *
     * <p>This is the total current limit for the sim
     *
     * @param currentLimit the current limit for the motor
     * @return
     */
    public SimMotorConfigs withStatorCurrentLimit(Current currentLimit) {
        // this is a limit across the sum of all motors output,
        // so it should be set to the total current limit of the mechanism
        this.currentLimit = currentLimit;
        return this;
    }

    /**
     * Configures the hard limits for the motor.
     *
     * @param forwardLimit the forward limit
     * @param reverseLimit the reverse limit
     * @return this instance for method chaining
     */
    public SimMotorConfigs withHardLimits(Angle forwardLimit, Angle reverseLimit) {
        this.forwardHardwareLimit = forwardLimit;
        this.reverseHardwareLimit = reverseLimit;
        return this;
    }
}
