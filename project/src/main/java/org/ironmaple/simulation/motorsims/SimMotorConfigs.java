package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.SimulatedArena;

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
     * <h4>All the controllers receives <strong>final</strong> position of the motor, using
     * {@link SimMotorState#finalAngularPosition()} and {@link SimMotorState#finalAngularVelocity()}.</h4>
     */
    public final PIDController positionVoltageController,
            velocityVoltageController,
            positionCurrentController,
            velocityCurrentController;

    public SimpleMotorFeedforward feedforward;

    protected Angle forwardHardwareLimit, reverseHardwareLimit, forwardSoftwareLimit, reverseSoftwareLimit;
    protected Current currentLimit;

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
        this.positionVoltageController = new PIDController(0, 0, 0);
        this.positionCurrentController = new PIDController(0, 0, 0);
        this.velocityVoltageController = new PIDController(0, 0, 0);
        this.velocityCurrentController = new PIDController(0, 0, 0);
        this.feedforward = new SimpleMotorFeedforward(0, 0);
        this.forwardHardwareLimit = this.forwardSoftwareLimit = Radians.of(Double.POSITIVE_INFINITY);
        this.reverseHardwareLimit = this.reverseSoftwareLimit = Radians.of(Double.NEGATIVE_INFINITY);
        this.currentLimit = Amps.of(150);
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
                .divide(gearing);
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
     * <h2>Configures the feed-forward calculator for the motor.</h2>
     *
     * <p>This method sets up a feed-forward calculator for the motor, which helps in estimating the required voltage to
     * achieve a desired motor output, based on the current state of the system. The feed-forward components include the
     * static voltage offset (`kS`), velocity-dependent voltage (`kV`), and acceleration-dependent voltage (`kA`).
     *
     * <p>The feed-forward calculator is configured with the following parameters:
     *
     * @param kS the static voltage offset for the motor.
     * @param kV the velocity-dependent voltage coefficient for the motor, <b>in voltage / un-geared motor velocity</b>.
     * @param kA the acceleration-dependent voltage coefficient for the motor, <b>in voltage / un-geared motor
     *     acceleration</b>.
     * @param dt the time step used for simulation.
     * @return the updated {@link SimMotorConfigs} object with the specified feed-forward calculator configured.
     */
    public SimMotorConfigs withFeedForward(
            Voltage kS,
            Per<VoltageUnit, AngularVelocityUnit> kV,
            Per<VoltageUnit, AngularAccelerationUnit> kA,
            Time dt) {
        var kVUnit = PerUnit.combine(Volts, RadiansPerSecond);
        var kAUnit = PerUnit.combine(Volts, RadiansPerSecondPerSecond);
        this.feedforward = new SimpleMotorFeedforward(
                kS.in(Volts), kV.in(kVUnit) * gearing, kA.in(kAUnit) * gearing, dt.in(Seconds));
        return this;
    }

    /**
     *
     *
     * <h2>Configures a default set of feed-forward gains.</h2>
     *
     * <p>The feed-forward gains (`ks`, `kv`, and `ka`) are automatically calculated using the data from the
     * {@link DCMotor} model.
     *
     * @return this instance for method chaining, with the default feed-forward gains configured.
     */
    public SimMotorConfigs withDefaultFeedForward() {
        return this.withFeedForward(
                this.calculateVoltage(calculateCurrent(friction), RadiansPerSecond.zero()),
                VoltsPerRadianPerSecond.ofNative(motor.nominalVoltageVolts / motor.freeSpeedRadPerSec),
                VoltsPerRadianPerSecondSquared.ofNative(motor.nominalVoltageVolts
                        / (motor.stallTorqueNewtonMeters / this.loadMOI.in(KilogramSquareMeters))),
                Seconds.of(SimulatedArena.getSimulationDt()));
    }

    /**
     *
     *
     * <h2>Configures the PD controller for {@link ControlRequest.PositionVoltage} requests.</h2>
     *
     * <p>For example, this can be configured as follows:
     *
     * <pre><code>
     * // Volts per Rotation of error is how CTRE handles PID when used with voltage requests
     * sim.withPositionalVoltageController(
     *   Volts.per(Rotation).ofNative(100.0),
     *   Volts.per(RotationsPerSecond).ofNative(5.0)
     * );
     * </code></pre>
     *
     * @param kP the proportional gain, <b>in voltage / final rotter position</b>
     * @param kD the derivative gain, <b>in voltage / final rotter velocity</b>
     * @return this instance for method chaining
     */
    public SimMotorConfigs withPositionVoltageController(
            Per<VoltageUnit, AngleUnit> kP, Per<VoltageUnit, AngularVelocityUnit> kD) {
        var kPUnit = PerUnit.combine(Volts, Radians);
        var kDUnit = PerUnit.combine(Volts, RadiansPerSecond);
        positionVoltageController.setP(kP.in(kPUnit));
        positionVoltageController.setD(kD.in(kDUnit));
        return this;
    }

    /**
     *
     *
     * <h2>Configures the PD controller for {@link ControlRequest.VelocityVoltage} requests.</h2>
     *
     * <p>For example, it can be configured as follows:
     *
     * <pre><code>
     * // Volts per RPS of error is how CTRE handles PID when used with velocity requests
     * sim.withVelocityVoltageController(
     *   Volts.per(RotationsPerSecond).ofNative(0.4)
     * );
     * </code></pre>
     *
     * @param kP the proportional gain, <b>in voltage / final rotter velocity</b>
     * @return this instance for method chaining
     */
    public SimMotorConfigs withVelocityVoltageController(Per<VoltageUnit, AngularVelocityUnit> kP) {
        var kPUnit = PerUnit.combine(Volts, RadiansPerSecond);
        velocityVoltageController.setP(kP.in(kPUnit));
        return this;
    }

    /**
     *
     *
     * <h2>Configures the PD controller for {@link ControlRequest.PositionCurrent} requests.</h2>
     *
     * <p>For example, it can be configured as follows:
     *
     * <pre><code>
     * // Amps per Rotation of error is how CTRE handles PID when used with current requests
     * sim.withPositionalCurrentController(
     *   Amps.per(Rotation).ofNative(100.0),
     *   Amps.per(RotationsPerSecond).ofNative(5.0)
     * );
     * </code></pre>
     *
     * @param kP the proportional gain, <b>in current / final rotter position</b>
     * @param kD the derivative gain, <b>in current / final rotter velocity</b>
     * @return this instance for method chaining
     */
    public SimMotorConfigs withPositionCurrentController(
            Per<CurrentUnit, AngleUnit> kP, Per<CurrentUnit, AngularVelocityUnit> kD) {
        var kPUnit = PerUnit.combine(Amps, Radians);
        var kDUnit = PerUnit.combine(Amps, RadiansPerSecond);
        positionCurrentController.setP(kP.in(kPUnit));
        positionCurrentController.setD(kD.in(kDUnit));
        return this;
    }

    /**
     *
     *
     * <h2>Configures the PD controller for {@link ControlRequest.VelocityCurrent} requests using current commands.</h2>
     *
     * <p>For example, it can be configured as follows:
     *
     * <pre><code>
     * // Amps per RPS of error is how CTRE handles PID when used with current requests
     * sim.withVelocityCurrentController(
     *   Amps.per(RotationsPerSecond).ofNative(0.4)
     * );
     * </code></pre>
     *
     * @param kP the proportional gain, <b>in current / final rotter velocity</b>
     * @return this instance for method chaining
     */
    public SimMotorConfigs withVelocityCurrentController(Per<CurrentUnit, AngularVelocityUnit> kP) {
        var kPUnit = PerUnit.combine(Amps, RadiansPerSecond);
        velocityCurrentController.setP(kP.in(kPUnit));
        return this;
    }

    /**
     *
     *
     * <h2>Configures the positional controllers to use continuous wrap.</h2>
     *
     * <p>It is typically used in applications where the input can exceed its normal range and you want the controller
     * to handle these cases seamlessly (e.g., in rotational systems where angles are cyclic).
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
     *
     *
     * <h2>Configures the current limit for the motor.</h2>
     *
     * <p>This method sets the total current limit for the motor's stator. The limit is applied during the simulation to
     * ensure realistic behavior and prevent simulation errors.
     *
     * @param currentLimit the current limit for the motor, typically expressed in amps
     * @return this instance for method chaining
     */
    public SimMotorConfigs withStatorCurrentLimit(Current currentLimit) {
        // this is a limit across the sum of all motors output,
        // so it should be set to the total current limit of the mechanism
        this.currentLimit = currentLimit;
        return this;
    }

    /**
     *
     *
     * <h2>Configures the software limits for the motor.</h2>
     *
     * <p>This method sets the software limits for the motor's movement. When either the forward or reverse limit is
     * reached, the motor will stop applying voltage in that direction, effectively preventing it from exceeding the
     * specified range.
     *
     * @param forwardLimit the forward limit angle, beyond which the motor will not apply voltage
     * @param reverseLimit the reverse limit angle, beyond which the motor will not apply voltage
     * @return this instance for method chaining
     */
    public SimMotorConfigs withSoftLimits(Angle forwardLimit, Angle reverseLimit) {
        this.forwardSoftwareLimit = forwardLimit;
        this.reverseSoftwareLimit = reverseLimit;
        return this;
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
                .withPositionVoltageController(
                        Volts.per(Radians).ofNative(positionVoltageController.getP()),
                        Volts.per(RadiansPerSecond).ofNative(positionVoltageController.getD()))
                .withVelocityVoltageController(Volts.per(RadiansPerSecond).ofNative(velocityVoltageController.getP()))
                .withPositionCurrentController(
                        Amps.per(Radians).ofNative(positionCurrentController.getP()),
                        Amps.per(RadiansPerSecond).ofNative(positionCurrentController.getD()))
                .withVelocityCurrentController(Amps.per(RadiansPerSecond).ofNative(velocityCurrentController.getP()));

        if (positionVoltageController.isContinuousInputEnabled()) cfg.withControllerContinousInput();

        return cfg;
    }
}
