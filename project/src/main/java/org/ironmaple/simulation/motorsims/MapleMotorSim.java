package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.utils.mathutils.MapleCommonMath;

/**
 *
 *
 * <h1>{@link edu.wpi.first.wpilibj.simulation.DCMotorSim} with a bit of extra spice.</h1>
 *
 * <p>This class extends the functionality of the original {@link edu.wpi.first.wpilibj.simulation.DCMotorSim} and
 * models the following aspects in addition:
 *
 * <ul>
 *   <li>Motor Controller Closed Loops.
 *   <li>Smart current limiting.
 *   <li>Friction force on the rotor.
 * </ul>
 */
public class MapleMotorSim {
    private final SimMotorConfigs configs;

    private SimMotorState state;
    private ControlRequest request;
    private Voltage appliedVoltage;
    private Current statorCurrent;

    /**
     *
     *
     * <h2>Constructs a Brushless Motor Simulation Instance.</h2>
     *
     * @param configs the configuration for this motor
     */
    public MapleMotorSim(SimMotorConfigs configs) {
        this.configs = configs;
        this.state = new SimMotorState(Radians.zero(), RadiansPerSecond.zero());
        this.request = new ControlRequest.VoltageOut(Volts.zero());
        this.appliedVoltage = Volts.zero();
        this.statorCurrent = Amps.zero();

        SimulatedBattery.getInstance().addMotor(this);
    }

    /**
     *
     *
     * <h2>Updates the simulation.</h2>
     *
     * <p>This is equivalent to{@link edu.wpi.first.wpilibj.simulation.DCMotorSim#update(double)}.
     */
    public void update(Time dt) {
        final Angle motorAndEncoderPosition = state.finalAngularPosition().times(configs.gearing);
        final AngularVelocity motorAndEncoderVelocity =
                state.finalAngularVelocity().times(configs.gearing);
        this.appliedVoltage = constrainOutputVoltage(
                state, request.updateSignal(configs, motorAndEncoderPosition, motorAndEncoderVelocity), configs);
        this.statorCurrent = configs.calculateCurrent(motorAndEncoderVelocity, appliedVoltage);
        this.state = this.state.step(
                configs.calculateTorque(statorCurrent).times(configs.gearing), configs.friction, configs.loadMOI, dt);

        if (state.finalAngularPosition().lte(configs.reverseHardwareLimit))
            state = new SimMotorState(configs.reverseHardwareLimit, RadiansPerSecond.zero());
        else if (state.finalAngularPosition().gte(configs.forwardHardwareLimit))
            state = new SimMotorState(configs.forwardHardwareLimit, RadiansPerSecond.zero());
    }

    /**
     *
     *
     * <h2>Requests an Output for the motor</h2>
     *
     * @param request a {@link ControlRequest} instance yielding a requested output
     */
    public void requestOutput(ControlRequest request) {
        this.request = request;
    }

    /**
     *
     *
     * <h2>(Utility Function) Constrains the Output Voltage of a Motor.</h2>
     *
     * <p>Constrains the output voltage of a motor such that the <strong>stator</strong> current does not exceed the
     * limit configured in {@link SimMotorConfigs#withStatorCurrentLimit(Current)}.
     *
     * @param state the {@link SimMotorState} of the motor
     * @param requestedVoltage the requested voltage
     * @param configs the configuration for the motor
     * @return the constrained voltage that satisfied the limits
     */
    public static Voltage constrainOutputVoltage(
            SimMotorState state, Voltage requestedVoltage, SimMotorConfigs configs) {
        final double kCurrentThreshold = 1.2;

        // don't use WpiLib Units for calculations
        final double motorCurrentVelocityRadPerSec =
                state.finalAngularVelocity().in(RadiansPerSecond) * configs.gearing;
        final double currentLimitAmps = configs.currentLimit.in(Amps);
        final double requestedOutputVoltageVolts = requestedVoltage.in(Volts);
        final double currentAtRequestedVoltageAmps =
                configs.motor.getCurrent(motorCurrentVelocityRadPerSec, requestedOutputVoltageVolts);

        // Resource for current limiting:
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
        final boolean currentTooHigh = currentAtRequestedVoltageAmps > (kCurrentThreshold * currentLimitAmps);
        final double limitedCurrent =
                MapleCommonMath.constrainMagnitude(currentAtRequestedVoltageAmps, currentLimitAmps);
        double limitedVoltage =
                configs.motor.getVoltage(configs.motor.getTorque(limitedCurrent), motorCurrentVelocityRadPerSec);

        // ensure the current limit doesn't cause an increase to output voltage
        if (Math.abs(limitedVoltage) > Math.abs(requestedOutputVoltageVolts))
            limitedVoltage = requestedOutputVoltageVolts;

        // apply software limits
        if (state.finalAngularPosition().gte(configs.forwardSoftwareLimit) && limitedVoltage > 0) limitedVoltage = 0;
        if (state.finalAngularPosition().lte(configs.reverseHardwareLimit) && limitedVoltage < 0) limitedVoltage = 0;

        // constrain the output voltage to the battery voltage
        return Volts.of(limitedVoltage);
    }

    /**
     *
     *
     * <h2>Obtains the <strong>final</strong> position of the rotter.</h2>
     *
     * <p>This is equivalent to {@link edu.wpi.first.wpilibj.simulation.DCMotorSim#getAngularPosition()}.
     *
     * @return the angular position of the motor, continuous
     */
    public Angle getAngularPosition() {
        return state.finalAngularPosition();
    }

    /**
     *
     *
     * <h2>Obtains the angular position measured by the relative encoder of the motor.</h2>
     *
     * @return the angular position measured by the encoder, continuous
     */
    public Angle getEncoderPosition() {
        return getAngularPosition().times(configs.gearing);
    }

    /**
     *
     *
     * <h2>Obtains the <strong>final</strong> velocity of the rotter.</h2>
     *
     * <p>This is equivalent to {@link edu.wpi.first.wpilibj.simulation.DCMotorSim#getAngularVelocity()}.
     *
     * @return the final angular velocity of the rotter
     */
    public AngularVelocity getVelocity() {
        return state.finalAngularVelocity();
    }

    /**
     *
     *
     * <h2>Obtains the angular velocity measured by the relative encoder of the motor.</h2>
     *
     * @return the angular velocity measured by the encoder
     */
    public AngularVelocity getEncoderVelocity() {
        return getVelocity().times(configs.gearing);
    }

    /**
     *
     *
     * <h2>Obtains the applied voltage by the motor controller.</h2>
     *
     * <p>The applied voltage is calculated by the motor controller in the previous call to {@link #update(Time)}
     *
     * <p>The control request specified by {@link #requestOutput(ControlRequest)} is used to calculate the applied
     * voltage.
     *
     * <p>The applied voltage is also restricted for current limit and battery voltage.
     *
     * @return the applied voltage
     */
    public Voltage getAppliedVoltage() {
        return appliedVoltage;
    }

    /**
     *
     *
     * <h2>Obtains the <strong>stator</strong> current.</h2>
     *
     * <p>This is equivalent to {@link DCMotorSim#getCurrentDrawAmps()}
     *
     * @return the stator current of the motor
     */
    public Current getStatorCurrent() {
        return statorCurrent;
    }

    /**
     *
     *
     * <h2>Obtains the <strong>supply</strong> current.</h2>
     *
     * <p>The supply current is different from the stator current, as described <a
     * href='https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10'>here</a>.
     *
     * @return the supply current of the motor
     */
    public Current getSupplyCurrent() {
        // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10
        return getStatorCurrent().times(appliedVoltage.divide(Volts.of(12)));
    }

    /**
     *
     *
     * <h2>Obtains the configuration of the motor.</h2>
     *
     * <p>You can modify the configuration of this motor by:
     *
     * <pre><code>
     *     mapleMotorSim.getConfigs()
     *          .with...(...)
     *          .with...(...);
     * </code></pre>
     *
     * @return the configuration of the motor
     */
    public SimMotorConfigs getConfigs() {
        return this.configs;
    }
}
