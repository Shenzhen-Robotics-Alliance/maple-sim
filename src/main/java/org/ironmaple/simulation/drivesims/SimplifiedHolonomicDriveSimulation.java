package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.ironmaple.utils.commands.HolonomicDriveSubsystem;

import static org.ironmaple.utils.mathutils.MapleCommonMath.constrainMagnitude;

/**
 * <p>Simulates a SIMPLIFIED holonomic drivetrain. </p>
 * <p>The drivetrain has collision space, and can drive on the field. </p>
 * <p>But it feels kind of like a Mecanum drive. </p>
 * <p>Used to simulate AI cycle bots or opponent defense bots. </p>
 * <p>For a more realistic simulation, it is strongly recommend to use {@link SwerveDriveSimulation} to simulate the main robot. </p>
 * */
public class SimplifiedHolonomicDriveSimulation extends AbstractDriveTrainSimulation implements HolonomicDriveSubsystem {
    public SimplifiedHolonomicDriveSimulation(DriveTrainSimulationProfile profile, Pose2d initialPoseOnField) {
        super(profile, initialPoseOnField);
    }

    /**
     * get the current pose
     * */
    @Override
    public Pose2d getPose() {
        return super.getSimulatedDriveTrainPose();
    }

    /**
     * <p>You CANNOT set the odometry pose for a simplified holonomic drive simulation.</p>
     * <p>Since the odometry is ALWAYS correct (odometry skidding is not simulated in this class)</p>
     * */
    @Deprecated @Override public void setPose(Pose2d currentPose) {}

    @Override
    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return getDriveTrainSimulatedChassisSpeedsRobotRelative();
    }

    @Override public double getChassisMaxLinearVelocityMetersPerSec() {return profile.maxLinearVelocity; }
    @Override public double getChassisMaxAccelerationMetersPerSecSq() {return profile.maxLinearAcceleration; }
    @Override public double getChassisMaxAngularVelocity() {return profile.maxAngularVelocity; }
    @Override public double getChassisMaxAngularAccelerationRadPerSecSq() {return profile.maxAngularAcceleration; }

    private ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds();


    @Override
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        desiredFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getFacing());
    }

    @Override
    public void simulationPeriodic(int iterations, double periodSeconds) {
        simulateChassisBehaviorWithFieldRelativeSpeeds(desiredFieldRelativeSpeeds);
    }

    private void simulateChassisBehaviorWithFieldRelativeSpeeds(ChassisSpeeds desiredChassisSpeedsFieldRelative) {
        super.setAtRest(false);

        final Vector2 desiredLinearMotionPercent = GeometryConvertor
                .toDyn4jLinearVelocity(desiredChassisSpeedsFieldRelative)
                .multiply(1.0/ profile.maxLinearVelocity);
        simulateChassisTranslationalBehavior(Vector2.create(
                constrainMagnitude(desiredLinearMotionPercent.getMagnitude(), 1),
                desiredLinearMotionPercent.getDirection()
        ));

        final double desiredRotationalMotionPercent = desiredChassisSpeedsFieldRelative.omegaRadiansPerSecond / profile.maxAngularVelocity;
        simulateChassisRotationalBehavior(constrainMagnitude(desiredRotationalMotionPercent, 1));
    }

    private void simulateChassisTranslationalBehavior(Vector2 desiredLinearMotionPercent) {
        final boolean robotRequestedToMoveLinearly = desiredLinearMotionPercent.getMagnitude() > 0.03;
        final Vector2 forceVec = desiredLinearMotionPercent.copy()
                .multiply(this.profile.robotMass * this.profile.maxLinearAcceleration);

        if (robotRequestedToMoveLinearly)
            super.applyForce(new Force(forceVec));
        else
            simulateChassisLinearFriction();
    }

    private void simulateChassisRotationalBehavior(double desiredRotationalMotionPercent) {
        final double maximumTorque = this.profile.maxAngularAcceleration * super.getMass().getInertia();
        if (Math.abs(desiredRotationalMotionPercent) > 0.01)
            super.applyTorque(desiredRotationalMotionPercent * maximumTorque);
        else
            simulateChassisAngularFriction();
    }
}
