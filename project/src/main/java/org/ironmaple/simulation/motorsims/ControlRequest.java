package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

/**
 *
 *
 * <h2>Represents a control request for a {@link MapleMotorSim} motor simulator.</h2>
 *
 * <p>A control request defines the behavior of a controller running on a {@link MapleMotorSim} instance to achieve a
 * specific task. The control request can either be open-loop or closed-loop, depending on the type of controller used.
 *
 * <p>The following control requests are supported:
 *
 * <ul>
 *   <li>{@link VoltageOut} A request to output a constant voltage to the motor.
 *   <li>{@link CurrentOut} A request to output a constant current to the motor.
 *   <li>{@link PositionVoltage} A request to control the motor's position using a voltage controller.
 *   <li>{@link PositionCurrent} A request to control the motor's position using a current controller.
 *   <li>{@link VelocityVoltage} A request to control the motor's velocity using a voltage controller.
 *   <li>{@link VelocityCurrent} A request to control the motor's velocity using a current controller.
 * </ul>
 */
public sealed interface ControlRequest {

    /**
     *
     *
     * <h2>Updates the control signal based on the motor's state.</h2>
     *
     * <p>Given the motor's current configuration and state (angular position and velocity), this method calculates the
     * control signal (voltage) that should be applied to the motor.
     *
     * @param configs the motor's configuration
     * @param mechanismAngle the final position of the mechanism
     * @param mechanismVelocity the final velocity of the mechanism
     * @return the updated control signal in volts
     */
    Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity);

    /**
     *
     *
     * <h2>Represents a constant voltage output.</h2>
     *
     * <p>This control request outputs a constant voltage to the motor, regardless of the motor's position or velocity.
     *
     * @param voltage the requested voltage to apply to the motor
     */
    record VoltageOut(Voltage voltage) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity) {
            return voltage;
        }
    }

    /**
     *
     *
     * <h2>Represents a constant current output.</h2>
     *
     * <p>This control request outputs a constant current to the motor. The voltage required to produce the current is
     * calculated based on the motor's configuration and velocity.
     *
     * @param current the requested current to apply to the motor
     */
    record CurrentOut(Current current) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity) {
            return configs.calculateVoltage(current, mechanismVelocity);
        }
    }

    /**
     *
     *
     * <h2>Represents a position control request with voltage control.</h2>
     *
     * <p>This control request uses a closed-loop voltage controller to drive the motor to a specified position.
     *
     * <p>The controller needs to be pre-configured through {@link SimMotorConfigs#withPositionVoltageController(Per,
     * Per)}.
     *
     * @param setPoint the target position in encoder angle
     */
    record PositionVoltage(Angle setPoint) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity) {
            // Compute the control signal (voltage) to achieve the target position
            Voltage voltage = Volts.of(
                    configs.positionVoltageController.calculate(mechanismAngle.in(Radians), setPoint.in(Radians)));

            // Calculate the feedforward voltage to account for known motor characteristics
            Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(
                    configs.calculateVelocity(configs.calculateCurrent(mechanismVelocity, voltage), voltage)
                            .in(RadiansPerSecond)));

            // Return the sum of feedback and feedforward voltages
            return feedforwardVoltage.plus(voltage);
        }
    }

    /**
     *
     *
     * <h2>Represents a position control request with current control.</h2>
     *
     * <p>This control request uses a closed-loop current controller to drive the motor to a specified position.
     *
     * <p>The controller needs to be pre-configured through {@link SimMotorConfigs#withPositionCurrentController(Per,
     * Per)}.
     *
     * @param setPoint the target position in encoder angle
     */
    record PositionCurrent(Angle setPoint) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity) {
            // Calculate the current required to achieve the target position
            Current current = Amps.of(
                    configs.positionCurrentController.calculate(mechanismAngle.in(Radians), setPoint.in(Radians)));

            // Calculate the voltage required to apply the current
            Voltage currentVoltage = configs.calculateVoltage(current, mechanismVelocity);

            // Calculate the feedforward voltage based on the motor's velocity
            Voltage feedforwardVoltage =
                    Volts.of(configs.feedforward.calculate(mechanismVelocity.in(RadiansPerSecond)));

            // Return the sum of the voltage and feedforward voltages
            return currentVoltage.plus(feedforwardVoltage);
        }
    }

    /**
     *
     *
     * <h2>Represents a velocity control request with voltage control.</h2>
     *
     * <p>This control request uses a closed-loop voltage controller to drive the motor to a specified velocity.
     *
     * <p>The controller needs to be pre-configured through {@link SimMotorConfigs#withVelocityVoltageController(Per)}.
     *
     * @param setPoint the target velocity in encoder velocity (rotations per second)
     */
    record VelocityVoltage(AngularVelocity setPoint) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity) {
            // Calculate the feedback voltage to drive the motor to the target velocity
            Voltage feedbackVoltage = Volts.of(configs.velocityVoltageController.calculate(
                    mechanismVelocity.in(RadiansPerSecond), setPoint.in(RadiansPerSecond)));

            // Calculate the feedforward voltage based on the target velocity
            Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(setPoint.in(RadiansPerSecond)));

            // Return the sum of feedback and feedforward voltages
            return feedbackVoltage.plus(feedforwardVoltage.copy());
        }
    }

    /**
     *
     *
     * <h2>Represents a velocity control request with current control.</h2>
     *
     * <p>This control request uses a closed-loop current controller to drive the motor to a specified velocity.
     *
     * <p>The controller needs to be pre-configured through {@link SimMotorConfigs#withVelocityCurrentController(Per)}.
     *
     * @param setPoint the target velocity in encoder velocity (rotations per second)
     */
    record VelocityCurrent(AngularVelocity setPoint) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity) {
            // Calculate the current required to achieve the target velocity
            Current current = Amps.of(configs.velocityCurrentController.calculate(
                    mechanismVelocity.in(RadiansPerSecond), setPoint.in(RadiansPerSecond)));

            // Calculate the voltage required to apply the current
            Voltage currentVoltage = configs.calculateVoltage(current, mechanismVelocity);

            // Calculate the feedforward voltage based on the target velocity
            Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(setPoint.in(RadiansPerSecond)));

            // Return the sum of the current and feedforward voltages
            return currentVoltage.plus(feedforwardVoltage);
        }
    }
}
