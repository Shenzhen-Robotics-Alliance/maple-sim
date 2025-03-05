package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
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
 * </ul>
 */
public class MapleMotorSim {
    private final SimMotorConfigs configs;
    private final DCMotorSim motorSim;

    private SimulatedMotorController controller;
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
        this.motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        configs.motor, configs.loadMOI.in(KilogramSquareMeters), configs.gearing),
                configs.motor);
        this.controller = (mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity) -> Volts.of(0);
        this.appliedVoltage = Volts.zero();
        this.statorCurrent = Amps.zero();

        SimulatedBattery.addMotor(this);
    }

    /**
     *
     *
     * <h2>Updates the simulation.</h2>
     *
     * <p>This is equivalent to{@link edu.wpi.first.wpilibj.simulation.DCMotorSim#update(double)}.
     */
    public void update(Time dt) {
        this.appliedVoltage = controller.updateControlSignal(
                motorSim.getAngularPosition(),
                motorSim.getAngularVelocity(),
                motorSim.getAngularPosition().times(configs.gearing),
                motorSim.getAngularVelocity().times(configs.gearing));
        this.appliedVoltage = SimulatedBattery.clamp(appliedVoltage);
        motorSim.setInputVoltage(appliedVoltage.in(Volts));
        motorSim.update(dt.in(Seconds));

        if (motorSim.getAngularPosition().lte(configs.reverseHardwareLimit))
            motorSim.setAngle(configs.reverseHardwareLimit.in(Radians));
        else if (motorSim.getAngularPosition().gte(configs.forwardHardwareLimit))
            motorSim.setAngle(configs.forwardHardwareLimit.in(Radians));
    }

    public <T extends SimulatedMotorController> T useMotorController(T motorController) {
        this.controller = motorController;
        return motorController;
    }

    public SimulatedMotorController.GenericMotorController useSimpleDCMotorController() {
        return useMotorController(new SimulatedMotorController.GenericMotorController(configs.motor));
    }

    /**
     *
     *
     * <h2>Obtains the <strong>final</strong> position of the mechanism.</h2>
     *
     * <p>This is equivalent to {@link edu.wpi.first.wpilibj.simulation.DCMotorSim#getAngularPosition()}.
     *
     * @return the angular position of the mechanism, continuous
     */
    public Angle getAngularPosition() {
        return motorSim.getAngularPosition();
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
     * @return the final angular velocity of the mechanism
     */
    public AngularVelocity getVelocity() {
        return motorSim.getAngularVelocity();
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
     * <p>The motor controller specified by {@link #useMotorController(SimulatedMotorController)} is used to calculate
     * the applied voltage.
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
        return getStatorCurrent().times(appliedVoltage.div(SimulatedBattery.getBatteryVoltage()));
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
