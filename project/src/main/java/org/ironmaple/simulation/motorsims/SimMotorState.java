package org.ironmaple.simulation.motorsims;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public record SimMotorState(Angle angularPosition, AngularVelocity angularVelocity) {
    public SimMotorState step(Torque electricTorque, Torque frictionTorque, MomentOfInertia loadMOI, Time dt) {
        // step0: convert all WPILib units to SI units
        double currentAngularPositionRadians = angularPosition.in(Radians);
        double currentAngularVelocityRadiansPerSecond = angularVelocity.in(RadiansPerSecond);
        final double electricTorqueNewtonsMeters = electricTorque.in(NewtonMeters);
        final double frictionTorqueNewtonsMeters = frictionTorque.in(NewtonMeters);
        final double loadMOIKgMetersSquared = loadMOI.in(KilogramSquareMeters);
        final double dtSeconds = dt.in(Seconds);

        // step1: apply electric torque
        currentAngularVelocityRadiansPerSecond += electricTorqueNewtonsMeters / loadMOIKgMetersSquared * dtSeconds;

        // step2: compute the amount of change in angular velocity due to friction
        final double deltaAngularVelocityDueToFrictionRadPerSec = Math.copySign(frictionTorqueNewtonsMeters, currentAngularVelocityRadiansPerSecond)
                / loadMOIKgMetersSquared * dtSeconds;

        // step3: if the angular velocity reverses direction after applying friction, or that the current angular velocity is zero
        if ((currentAngularPositionRadians + deltaAngularVelocityDueToFrictionRadPerSec) * currentAngularPositionRadians <= 0)
            currentAngularPositionRadians = 0;
        // step4: add the change in angular velocity due to friction
        else
            currentAngularPositionRadians += deltaAngularVelocityDueToFrictionRadPerSec;

        // step 5: indefinite integral on position
        currentAngularPositionRadians += currentAngularVelocityRadiansPerSecond * dtSeconds;
        return new SimMotorState(
                Radians.of(currentAngularPositionRadians),
                RadiansPerSecond.of(currentAngularVelocityRadiansPerSecond)
        );
    }
}
