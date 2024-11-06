package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
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
        this.appliedVoltage = constrainOutputVoltage(request.updateSignal(configs, state));
        this.statorCurrent = configs.calculateCurrent(state.angularVelocity(), appliedVoltage);
        this.state = this.state.step(configs.calculateTorque(statorCurrent), configs.friction, configs.loadMOI, dt);

        if (state.angularPosition().lte(configs.reverseHardwareLimit))
            state = new SimMotorState(configs.reverseHardwareLimit, RadiansPerSecond.zero());
        else if (state.angularPosition().gte(configs.forwardHardwareLimit))
            state = new SimMotorState(configs.forwardHardwareLimit, RadiansPerSecond.zero());
    }

    public void request(ControlRequest request) {
        this.request = request;
    }

    private Voltage constrainOutputVoltage(Voltage requestedVoltage) {
        final double kCurrentThreshold = 1.2;

        // don't use WpiLib Units for calculations
        final double motorCurrentVelocityRadPerSec = state.angularVelocity().in(RadiansPerSecond);
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

        // constrain the output voltage to the battery voltage
        return Volts.of(BatterySimulationContainer.getInstance().constrainVoltage(limitedVoltage));
    }

    public Angle getPosition() {
        return state.angularPosition();
    }

    public AngularVelocity getVelocity() {
        return state.angularVelocity();
    }

    public Voltage getAppliedVoltage() {
        return appliedVoltage;
    }

    public Current getStatorCurrent() {
        return statorCurrent;
    }

    public Current getSupplyCurrent() {
        // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10
        return getStatorCurrent().times(
                appliedVoltage.divide(Volts.of(RobotController.getBatteryVoltage()))
        );
    }
}
