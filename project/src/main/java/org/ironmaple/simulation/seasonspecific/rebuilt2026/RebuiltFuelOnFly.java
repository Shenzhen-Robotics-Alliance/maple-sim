package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

/**
 *
 *
 * <h1>Represents a FUEL launched into the air.</h1>
 *
 * <p>This class models a {@link RebuiltFuelOnField} launched into the air.
 *
 * <p>The simulation will determine if the FUEL hits its target—the HUB.
 *
 * <p>The user can specify a callback using {@link #setHitNetCallBack(Runnable)}, which will be triggered when the FUEL
 * hits the HUB.
 */
public class RebuiltFuelOnFly extends GamePieceProjectile {

    /**
     *
     *
     * <h2>Creates a FUEL Projectile Ejected from a Shooter.</h2>
     *
     * @param robotPosition the position of the robot (not the shooter) at the time of launching the FUEL
     * @param shooterPositionOnRobot the translation from the shooter's position to the robot's center, in the robot's
     *     frame of reference
     * @param chassisVelocitiesFieldRelative the field-relative velocity of the robot chassis when launching the FUEL,
     *     influencing the initial velocity of the FUEL
     * @param shooterFacing the direction in which the shooter is facing at launch
     * @param initialHeight the initial height of the FUEL when launched, i.e., the height of the shooter from the
     *     ground
     * @param launchingVelocity the velocity at which the FUEL is launched
     * @param shooterAngle the pitch angle of the shooter when launching
     */
    public RebuiltFuelOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisVelocities chassisVelocitiesFieldRelative,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingVelocity,
            Angle shooterAngle) {
        super(
                RebuiltFuelOnField.REBUILT_FUEL_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisVelocitiesFieldRelative,
                shooterFacing,
                initialHeight,
                launchingVelocity,
                shooterAngle);

        super.withTouchGroundHeight(Inches.of(3).in(Meters));
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }
}
