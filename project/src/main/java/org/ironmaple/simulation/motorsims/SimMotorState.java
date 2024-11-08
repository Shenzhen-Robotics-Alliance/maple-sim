package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public record SimMotorState(Angle finalAngularPosition, AngularVelocity finalAngularVelocity) {
    public SimMotorState step(
            Torque finalElectricTorque, Torque finalFrictionTorque, MomentOfInertia loadMOI, Time dt) {
        // step0: convert all WPILib units to SI units
        double currentAngularPositionRadians = finalAngularPosition.in(Radians);
        double currentAngularVelocityRadiansPerSecond = finalAngularVelocity.in(RadiansPerSecond);
        final double electricTorqueNewtonsMeters = finalElectricTorque.in(NewtonMeters);
        final double frictionTorqueNewtonsMeters = finalFrictionTorque.in(NewtonMeters);
        final double loadMOIKgMetersSquared = loadMOI.in(KilogramSquareMeters);
        final double dtSeconds = dt.in(Seconds);

        // step1: apply electric torque
        currentAngularVelocityRadiansPerSecond += electricTorqueNewtonsMeters / loadMOIKgMetersSquared * dtSeconds;

        // step2: compute the amount of change in angular velocity due to friction
        final double deltaAngularVelocityDueToFrictionRadPerSec =
                Math.copySign(frictionTorqueNewtonsMeters, -currentAngularVelocityRadiansPerSecond)
                        / loadMOIKgMetersSquared
                        * dtSeconds;

        // step3: if the angular velocity reverses direction after applying friction, or that the current angular
        // velocity is zero
        if ((currentAngularVelocityRadiansPerSecond + deltaAngularVelocityDueToFrictionRadPerSec)
                        * currentAngularVelocityRadiansPerSecond
                <= 0) currentAngularVelocityRadiansPerSecond = 0;
        // step4: add the change in angular velocity due to friction
        else currentAngularVelocityRadiansPerSecond += deltaAngularVelocityDueToFrictionRadPerSec;

        // step 5: indefinite integral on position
        currentAngularPositionRadians += currentAngularVelocityRadiansPerSecond * dtSeconds;
        return new SimMotorState(
                Radians.of(currentAngularPositionRadians), RadiansPerSecond.of(currentAngularVelocityRadiansPerSecond));
    }
}
