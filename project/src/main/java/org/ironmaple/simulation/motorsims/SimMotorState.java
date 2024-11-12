package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

/**
 *
 *
 * <h2>Represents the state of a simulated motor at a given point in time.</h2>
 *
 * <p>This record holds the final angular position and velocity of the motor. It is used to track the motor's state
 * during each simulation step.
 *
 * @param finalAngularPosition the final angular position of the motor, in radians
 * @param finalAngularVelocity the final angular velocity of the motor, in radians per second
 */
public record SimMotorState(Angle finalAngularPosition, AngularVelocity finalAngularVelocity) {
    /**
     *
     *
     * <h2>Simulates a step in the motor's motion based on the applied forces.</h2>
     *
     * <p>This method calculates the new angular position and velocity of the motor after applying electric and
     * frictional torques over a time step.
     *
     * <p>The method follows these steps:
     *
     * <ul>
     *   <li>Convert all units to SI units for calculation.
     *   <li>Apply the electric torque to the current angular velocity.
     *   <li>Compute the change in angular velocity due to friction.
     *   <li>If friction reverses the direction of angular velocity, the velocity is set to zero.
     *   <li>Integrate the angular velocity to find the new position.
     * </ul>
     *
     * @param finalElectricTorque the final applied electric torque, in Newton-meters
     * @param finalFrictionTorque the final frictional torque, in Newton-meters
     * @param loadMOI the moment of inertia of the load, in kilogram square meters
     * @param dt the time step for the simulation, in seconds
     * @return a new {@link SimMotorState} instance with the updated angular position and velocity
     */
    public SimMotorState step(
            Torque finalElectricTorque, Torque finalFrictionTorque, MomentOfInertia loadMOI, Time dt) {
        // Step 0: Convert all units to SI units (radians, radians per second, Newton-meters, seconds, kg*m²)
        double currentAngularPositionRadians = finalAngularPosition.in(Radians);
        double currentAngularVelocityRadiansPerSecond = finalAngularVelocity.in(RadiansPerSecond);
        final double electricTorqueNewtonsMeters = finalElectricTorque.in(NewtonMeters);
        final double frictionTorqueNewtonsMeters = finalFrictionTorque.in(NewtonMeters);
        final double loadMOIKgMetersSquared = loadMOI.in(KilogramSquareMeters);
        final double dtSeconds = dt.in(Seconds);

        // Step 1: Apply electric torque to the angular velocity.
        // The torque causes a change in the angular velocity, according to the moment of inertia.
        currentAngularVelocityRadiansPerSecond += electricTorqueNewtonsMeters / loadMOIKgMetersSquared * dtSeconds;

        // Step 2: Calculate the change in angular velocity due to friction.
        // Friction opposes the motion and reduces the angular velocity over time.
        final double deltaAngularVelocityDueToFrictionRadPerSec =
                Math.copySign(frictionTorqueNewtonsMeters, -currentAngularVelocityRadiansPerSecond)
                        / loadMOIKgMetersSquared
                        * dtSeconds;

        // Step 3: Check if the angular velocity changes direction due to friction, or if it reaches zero.
        // If friction causes the motor to reverse direction, or if the velocity reaches zero, set the angular velocity
        // to zero.
        if ((currentAngularVelocityRadiansPerSecond + deltaAngularVelocityDueToFrictionRadPerSec)
                        * currentAngularVelocityRadiansPerSecond
                <= 0)
            // The velocity has reversed direction or reached zero, so stop the motor
            currentAngularVelocityRadiansPerSecond = 0;
        else
            // Otherwise, apply the change due to friction
            currentAngularVelocityRadiansPerSecond += deltaAngularVelocityDueToFrictionRadPerSec;

        // Step 4: Integrate angular velocity to find the new position.
        // The new angular position is the current position plus the change in position over the time step.
        currentAngularPositionRadians += currentAngularVelocityRadiansPerSecond * dtSeconds;

        // Return a new instance with the updated angular position and velocity
        return new SimMotorState(
                Radians.of(currentAngularPositionRadians), RadiansPerSecond.of(currentAngularVelocityRadiansPerSecond));
    }
}
