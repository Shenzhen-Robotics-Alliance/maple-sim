package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.ironmaple.utils.commands.HolonomicDriveSubsystem;

import static org.ironmaple.utils.mathutils.MapleCommonMath.constrainMagnitude;

/**
 * <h1>Simulates a Simplified Holonomic Drivetrain.</h1>
 *
 * <p>This class simulates a simplified holonomic drivetrain with collision space, allowing the drivetrain to drive on the field.</p>
 * <p>It is suitable for simulating AI cycle bots or opponent defense bots.</p>
 * <p>However, this drivetrain simulation is simplified and not entirely realistic—while it accelerates smoothly, the kinematics are not fully accurate.</p>
 * <p>For a more realistic simulation of your robot's drivetrain, use {@link SwerveDriveSimulation} instead.</p>
 * */
public class SimplifiedHolonomicDriveSimulation extends AbstractDriveTrainSimulation implements HolonomicDriveSubsystem {
    /**
     * <h2>Creates a Simplified Holonomic Drivetrain Simulation.</h2>
     *
     * @param profile the profile containing all the configuration settings for the drivetrain, see {@link org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation.DriveTrainSimulationProfile}
     * @param initialPoseOnField the initial pose of the drivetrain in the simulation world
     */
    public SimplifiedHolonomicDriveSimulation(DriveTrainSimulationProfile profile, Pose2d initialPoseOnField) {
        super(profile, initialPoseOnField);
    }

    /**
     * <h2>Obtains the Position Estimated by the Odometry of This Simulated Chassis.</h2>
     *
     * <p>This method is used to enable AI opponent robots to follow cycle paths using <a href='https://pathplanner.dev/home.html'>Path-Planner</a>.</p>
     * <p>In this simplified simulation, odometry error is ignored, so the actual pose of the drivetrain is returned.</p>
     *
     * @return the real pose of the drivetrain.
     * */
    @Override
    public Pose2d getPose() {
        return super.getSimulatedDriveTrainPose();
    }

    private ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds();

    /**
     * <h2>Runs Chassis Speeds on This Simulated Chassis.</h2>
     *
     * <p>This method sets the <strong>DESIRED</strong> robot-relative chassis speeds on the simulated drivetrain.</p>
     * <p>Propelling forces will be applied to the chassis to help it accelerate smoothly to the desired speeds.</p>
     * <p>This method is different from {@link AbstractDriveTrainSimulation#setRobotSpeeds(ChassisSpeeds)}, which jumps to the speed <strong>Instantaneously</strong>.</p>
     *
     * @param speeds the desired robot-relative speeds, represented as {@link ChassisSpeeds}
     */
    @Override
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        desiredFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getFacing());
    }

    /**
     * <h2>Update this chassis simulation.</h2>
     * */
    @Override
    public void simulationSubTick() {
        simulateChassisBehaviorWithFieldRelativeSpeeds(desiredFieldRelativeSpeeds);
    }

    /**
     * <h2>Simulates the Chassis Behavior with Field-Relative Speeds.</h2>
     *
     * <p>This method updates the simulation with the given desired field-relative chassis speeds. It simulates both the linear force and rotational torque due to friction and propulsion.</p>
     *
     * @param desiredChassisSpeedsFieldRelative the desired chassis speeds relative to the field, represented as {@link ChassisSpeeds}
     * */
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

    /**
     * <h1>Simulates the Translational Behavior of the Chassis.</h1>
     *
     * <p>This method simulates the translational behavior of the chassis based on the desired percentage of linear motion the chassis is attempting to achieve.</p>
     *
     * @param desiredLinearMotionPercent the percentage of the linear motion the chassis is trying to reach, represented as a {@link Vector2}
     * */
    private void simulateChassisTranslationalBehavior(Vector2 desiredLinearMotionPercent) {
        final boolean robotRequestedToMoveLinearly = desiredLinearMotionPercent.getMagnitude() > 0.03;
        final Vector2 forceVec = desiredLinearMotionPercent.copy()
                .multiply(this.profile.robotMass * this.profile.maxLinearAcceleration);

        if (robotRequestedToMoveLinearly)
            super.applyForce(new Force(forceVec));
        else
            simulateChassisLinearFriction();
    }

    /**
     * <h2>Simulates the Rotational Behavior of the Chassis.</h2>
     *
     * <p>This method simulates the rotational behavior of the chassis based on the desired percentage of rotational motion the chassis is attempting to achieve.</p>
     *
     * @param desiredRotationalMotionPercent the percentage of the rotational motion the chassis is trying to reach, represented as a double
     * */
    private void simulateChassisRotationalBehavior(double desiredRotationalMotionPercent) {
        final double maximumTorque = this.profile.maxAngularAcceleration * super.getMass().getInertia();
        if (Math.abs(desiredRotationalMotionPercent) > 0.01)
            super.applyTorque(desiredRotationalMotionPercent * maximumTorque);
        else
            simulateChassisAngularFriction();
    }

    /**
     * <h2>Setting the Odometry Pose Is Not Supported for This Simplified Holonomic Drive Simulation.</h2>
     *
     * <p>The method executes nothing.</p>
     * <p>You cannot set the odometry pose since the odometry is always correct in this simulation.</p>
     * */
    @Deprecated @Override public void setPose(Pose2d currentPose) {}

    @Override
    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return super.getDriveTrainSimulatedChassisSpeedsRobotRelative();
    }

    @Override public double getChassisMaxLinearVelocityMetersPerSec() {return profile.maxLinearVelocity; }
    @Override public double getChassisMaxAccelerationMetersPerSecSq() {return profile.maxLinearAcceleration; }
    @Override public double getChassisMaxAngularVelocity() {return profile.maxAngularVelocity; }
    @Override public double getChassisMaxAngularAccelerationRadPerSecSq() {return profile.maxAngularAcceleration; }
}
