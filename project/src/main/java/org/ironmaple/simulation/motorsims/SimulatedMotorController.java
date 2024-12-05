package org.ironmaple.simulation.motorsims;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;

public interface SimulatedMotorController {
    Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle, AngularVelocity encoderVelocity);

    final class GenericDCMotorController implements SimulatedMotorController {
        public Current currentLimit = Amps.of(150);
        public Angle forwardSoftwareLimit = Radians.of(-Double.POSITIVE_INFINITY), reverseSoftwareLimit = Radians.of(Double.POSITIVE_INFINITY);

        private Voltage requestedVoltage = Volts.of(0);

        public void requestVoltage(Voltage voltage) {
            this.requestedVoltage = voltage;
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
         * @return the constrained voltage that satisfied the limits
         */
        public Voltage constrainOutputVoltage(SimMotorState state, Voltage requestedVoltage) {
            final double kCurrentThreshold = 1.2;

            // don't use WpiLib Units for calculations
            final double motorCurrentVelocityRadPerSec =
                    state.finalAngularVelocity().in(RadiansPerSecond) * gearing;
            final double currentLimitAmps = currentLimit.in(Amps);
            final double requestedOutputVoltageVolts = requestedVoltage.in(Volts);
            final double currentAtRequestedVoltageAmps =
                    motor.getCurrent(motorCurrentVelocityRadPerSec, requestedOutputVoltageVolts);

            // Resource for current limiting:
            // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
            double limitedVoltage = requestedOutputVoltageVolts;
            final boolean currentTooHigh = Math.abs(currentAtRequestedVoltageAmps) > (kCurrentThreshold * currentLimitAmps);
            if (currentTooHigh) {
                final double limitedCurrent = Math.copySign(currentLimitAmps, currentAtRequestedVoltageAmps);
                limitedVoltage = motor.getVoltage(motor.getTorque(limitedCurrent), motorCurrentVelocityRadPerSec);
            }

            // ensure the current limit doesn't cause an increase to output voltage
            if (Math.abs(limitedVoltage) > Math.abs(requestedOutputVoltageVolts))
                limitedVoltage = requestedOutputVoltageVolts;

            // apply software limits
            if (state.finalAngularPosition().gte(forwardSoftwareLimit) && limitedVoltage > 0) limitedVoltage = 0;
            if (state.finalAngularPosition().lte(reverseSoftwareLimit) && limitedVoltage < 0) limitedVoltage = 0;

            // constrain the output voltage to the battery voltage
            return Volts.of(limitedVoltage);
        }

        @Override
        public Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle, AngularVelocity encoderVelocity) {
            return null;
        }
    }
}
