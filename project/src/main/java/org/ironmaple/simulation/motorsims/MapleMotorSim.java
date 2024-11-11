package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.requests.ControlRequest;
import org.ironmaple.simulation.motorsims.requests.VoltageOut;
import org.ironmaple.utils.mathutils.MapleCommonMath;

/**
 *
 *
 * <h1>{@link DCMotorSim} with a bit of extra spice.</h1>
 *
 * <p>This class extends the functionality of the original {@link DCMotorSim} and models the following aspects in
 * addition:
 *
 * <ul>
 *   <li>Friction force on the rotor.
 *   <li>Smart current limiting.
 *   <li>Brake and coast modes (only for simulating brushless motors).
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
        this.request = new VoltageOut(Volts.zero());
        this.appliedVoltage = Volts.zero();
        this.statorCurrent = Amps.zero();

        BatterySimulationContainer.getInstance().addMotor(this);
    }

    public void update(Time dt) {
        final Angle motorAndEncoderPosition = state.finalAngularPosition().times(configs.gearing);
        final AngularVelocity motorAndEncoderVelocity =
                state.finalAngularVelocity().times(configs.gearing);
        this.appliedVoltage = constrainOutputVoltage(
                state.finalAngularVelocity(),
                request.updateSignal(configs, motorAndEncoderPosition, motorAndEncoderVelocity),
                configs);
        this.statorCurrent = configs.calculateCurrent(motorAndEncoderVelocity, appliedVoltage);
        this.state = this.state.step(
                configs.calculateTorque(statorCurrent).times(configs.gearing), configs.friction, configs.loadMOI, dt);

        if (state.finalAngularPosition().lte(configs.reverseHardwareLimit))
            state = new SimMotorState(configs.reverseHardwareLimit, RadiansPerSecond.zero());
        else if (state.finalAngularPosition().gte(configs.forwardHardwareLimit))
            state = new SimMotorState(configs.forwardHardwareLimit, RadiansPerSecond.zero());
    }

    public void requestOutput(ControlRequest request) {
        this.request = request;
    }

    public static Voltage constrainOutputVoltage(
            AngularVelocity currentVelocity, Voltage requestedVoltage, SimMotorConfigs configs) {
        final double kCurrentThreshold = 1.2;

        // don't use WpiLib Units for calculations
        final double motorCurrentVelocityRadPerSec = currentVelocity.in(RadiansPerSecond) * configs.gearing;
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
        if (state.finalAngularPosition().gte(configs.forwardSoftwareLimit) && limitedVoltage > 0)
            limitedVoltage = 0;
        if (state.finalAngularPosition().lte(configs.reverseHardwareLimit) && limitedVoltage < 0)
            limitedVoltage = 0;

        // constrain the output voltage to the battery voltage
        return Volts.of(BatterySimulationContainer.getInstance().constrainVoltage(limitedVoltage));
    }

    public Angle getPosition() {
        return state.finalAngularPosition();
    }

    public Angle getEncoderPosition() {
        return getPosition().times(configs.gearing);
    }

    public AngularVelocity getVelocity() {
        return state.finalAngularVelocity();
    }

    public AngularVelocity getEncoderVelocity() {
        return getVelocity().times(configs.gearing);
    }

    public Voltage getAppliedVoltage() {
        return appliedVoltage;
    }

    public Current getStatorCurrent() {
        return statorCurrent;
    }

    public Current getSupplyCurrent() {
        // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10
        return getStatorCurrent()
                .times(appliedVoltage.divide(
                        Volts.of(BatterySimulationContainer.getInstance().getBatteryVoltage())));
    }

    public SimMotorConfigs getConfigs() {
        return this.configs;
    }
}
