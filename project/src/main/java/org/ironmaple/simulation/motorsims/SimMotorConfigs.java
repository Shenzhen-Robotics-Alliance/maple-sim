package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    /**
     *
     *
     * <h3>All the controllers receives un-geared position from the <strong>encoder</strong>, using
     * {@link MapleMotorSim#getEncoderPosition()} and {@link MapleMotorSim#getEncoderVelocity()}.</h3>
     */
    public final PIDController positionVoltageController,
            velocityVoltageController,
            positionCurrentController,
            velocityCurrentController;

    public SimpleMotorFeedforward feedforward;

    protected Angle forwardHardwareLimit;
    protected Angle reverseHardwareLimit;
    protected Current currentLimit;

    /**
     *
     *
     * <h2>Constructs a simulated motor.</h2>
     *
     * @param (ChatGPT completes this part)
     */
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

    /**
     *
     *
     * <h2>Calculates the voltage of the motor.</h2>
     *
     * <p>This methods uses the {@link DCMotor} model to find the voltage for a given current and angular velocity.
     *
     * <p>It's the {@link DCMotor#getVoltage(double, double)} wrapped with WPILib units library.
     *
     * @param (ChaptGPT completes)
     * @return (ChatGPT completes)
     */
    public Voltage calculateVoltage(Current current, AngularVelocity angularVelocity) {
        // find what voltage is needed to get the current
        return Volts.of(motor.getVoltage(current.in(Amps), angularVelocity.in(RadiansPerSecond) * gearing));
    }

    // imitating the API docs above, ChatGPT completes the javadocs for the methods below:
    public AngularVelocity calculateVelocity(Current current, Voltage voltage) {
        return RadiansPerSecond.of(motor.getSpeed(motor.getTorque(current.in(Amps)), voltage.in(Volts)));
    }

    public Current calculateCurrent(AngularVelocity angularVelocity, Voltage voltage) {
        return Amps.of(motor.getCurrent(angularVelocity.in(RadiansPerSecond), voltage.in(Volts)));
    }

    public Current calculateCurrent(Torque torque) {
        return Amps.of(motor.getCurrent(torque.in(NewtonMeters)));
    }

    public Torque calculateTorque(Current current) {
        return NewtonMeters.of(motor.getTorque(current.in(Amps)));
    }

    /**
     *
     *
     * <h2>Configures the feed-forward calculator for the motor.</h2>
     *
     * GPT completes the params and returns
     */
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
     * Configures the positional controllers to use continuous wrap.
     *
     * @return this instance for method chaining
     * @see PIDController#enableContinuousInput(double, double)
     */
    public SimMotorConfigs withControllerContinousInput() {
        positionVoltageController.enableContinuousInput(0, 2 * Math.PI);
        positionCurrentController.enableContinuousInput(0, 2 * Math.PI);
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

    public SimMotorConfigs withSoftLimits(Angle forwardLimit, Angle reverseLimit) {
        this.forwardSoftwareLimit = forwardLimit;
        this.reverseSoftwareLimit = reverseLimit;
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

    @Override
    protected SimMotorConfigs clone() {
        SimMotorConfigs cfg = new SimMotorConfigs(
                        motor, gearing, loadMOI, Volts.of(motor.getVoltage(friction.in(NewtonMeter), 0.0)))
                .withFeedForward(
                        Volts.of(feedforward.getKs()),
                        VoltsPerRadianPerSecond.ofNative(feedforward.getKv()),
                        VoltsPerRadianPerSecondSquared.ofNative(feedforward.getKa()),
                        Seconds.of(feedforward.getDt()))
                .withHardLimits(forwardHardwareLimit, reverseHardwareLimit)
                .withStatorCurrentLimit(currentLimit)
                .withPositionalVoltageController(
                        Volts.per(Radians).ofNative(positionVoltageController.getP()),
                        Volts.per(RadiansPerSecond).ofNative(positionVoltageController.getD()))
                .withVelocityVoltageController(Volts.per(RadiansPerSecond).ofNative(velocityVoltageController.getP()))
                .withPositionalCurrentController(
                        Amps.per(Radians).ofNative(positionCurrentController.getP()),
                        Amps.per(RadiansPerSecond).ofNative(positionCurrentController.getD()))
                .withVelocityCurrentController(Amps.per(RadiansPerSecond).ofNative(velocityCurrentController.getP()));

        if (positionVoltageController.isContinuousInputEnabled()) cfg.withControllerContinousInput();

        return cfg;
    }
}
