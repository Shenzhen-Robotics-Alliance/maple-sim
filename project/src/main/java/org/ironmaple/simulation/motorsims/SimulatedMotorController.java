package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface SimulatedMotorController {
    Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity);

    final class GenericMotorController implements SimulatedMotorController {
        private final DCMotor model;
        private Current currentLimit = Amps.of(150);
        private Angle forwardSoftwareLimit = Radians.of(Double.POSITIVE_INFINITY),
                reverseSoftwareLimit = Radians.of(-Double.POSITIVE_INFINITY);

        private Voltage requestedVoltage = Volts.zero();
        private Voltage appliedVoltage = Volts.zero();

        public GenericMotorController(DCMotor model) {
            this.model = model;
        }

        public GenericMotorController withCurrentLimit(Current currentLimit) {
            this.currentLimit = currentLimit;
            return this;
        }

        public GenericMotorController withSoftwareLimits(Angle forwardSoftwareLimit, Angle reverseSoftwareLimit) {
            this.forwardSoftwareLimit = forwardSoftwareLimit;
            this.reverseSoftwareLimit = reverseSoftwareLimit;
            return this;
        }

        public void requestVoltage(Voltage voltage) {
            this.requestedVoltage = voltage;
        }

        /**
         *
         *
         * <h2>(Utility Function) Constrains the Output Voltage of a Motor.</h2>
         *
         * <p>Constrains the output voltage of a motor such that the <strong>stator</strong> current does not exceed the
         * current limit
         *
         * <p>Prevents motor from exceeding software limits
         *
         * @param encoderAngle the angle of the encoder
         * @param encoderVelocity the velocity of the encoder
         * @param requestedVoltage the requested voltage
         * @return the constrained voltage that satisfied the limits
         */
        public Voltage constrainOutputVoltage(
                Angle encoderAngle, AngularVelocity encoderVelocity, Voltage requestedVoltage) {
            // Don't use WPILib Units
            final double motorCurrentVelocityRadPerSec = encoderVelocity.in(RadiansPerSecond);
            final double requestedOutputVoltageVolts = requestedVoltage.in(Volts);
            final double currentLimitAmps = currentLimit.in(Amps);
            final double kCurrentThreshold = 1.2;
            final double thresholdedCurrentLimitAmps = kCurrentThreshold * currentLimitAmps;

            final double currentAtRequestedVoltageAmps =
                    model.getCurrent(motorCurrentVelocityRadPerSec, requestedOutputVoltageVolts);

            double limitedVoltage = requestedOutputVoltageVolts;

            // Resource for current limiting:
            // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
            if (Math.abs(currentAtRequestedVoltageAmps) > thresholdedCurrentLimitAmps) {
                final double limitedCurrent = Math.copySign(currentLimitAmps, currentAtRequestedVoltageAmps);
                limitedVoltage = model.getVoltage(model.getTorque(limitedCurrent), motorCurrentVelocityRadPerSec);

                // Ensure the current limit doesn't cause an increase to output voltage
                if (Math.abs(limitedVoltage) > Math.abs(requestedOutputVoltageVolts))
                    limitedVoltage = requestedOutputVoltageVolts;
            }

            // Apply software limits
            if (limitedVoltage > 0 && encoderAngle.gte(forwardSoftwareLimit)) limitedVoltage = 0;
            if (limitedVoltage < 0 && encoderAngle.lte(reverseSoftwareLimit)) limitedVoltage = 0;

            // Constrain the output voltage to the battery voltage
            return Volts.of(limitedVoltage);
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            appliedVoltage = constrainOutputVoltage(encoderAngle, encoderVelocity, requestedVoltage);
            return appliedVoltage;
        }

        public Voltage getAppliedVoltage() {
            return appliedVoltage;
        }
    }
}
