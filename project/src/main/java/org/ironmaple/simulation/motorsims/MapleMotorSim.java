package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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
        this.appliedVoltage = configs.constrainOutputVoltage(state, request.updateSignal(configs, state));
        this.statorCurrent = configs.calculateCurrent(state.finalAngularVelocity(), appliedVoltage);
        this.state = this.state.step(configs.calculateTorque(statorCurrent), configs.friction, configs.loadMOI, dt);

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
     * <h2>Obtains the <strong>final</strong> position of the mechanism.</h2>
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
     * <h2>Obtains the <strong>final</strong> velocity of the mechanism.</h2>
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
        // Supply Power = Stator Power (Conservation of Energy)
        // Hence,
        // Battery Voltage x Supply Current = Applied Voltage x Stator Current
        // Supply Current = Stator Current * Applied Voltage / Battery Voltage
        return getStatorCurrent().times(appliedVoltage.divide(Volts.of(12)));
    }

    public ControlRequest getRequestedControl() {
        return request;
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
