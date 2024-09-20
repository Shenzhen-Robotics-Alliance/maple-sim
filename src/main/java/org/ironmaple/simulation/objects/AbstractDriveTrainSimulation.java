package org.ironmaple.simulation.objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.utils.mathutils.GeometryConvertor;

/**
 * <p>Simulates an abstract drivetrain in the simulation world. </p>
 * <p>Simulates the mass, collision space and other physical properties of the drivetrain. </p>
 * <p>Simulates the friction forces of the drivetrain. </p>
 * <p>Propelling forces of the motors are simulated in other classes, such as {@link SimplifiedHolonomicDriveSimulation} or {@link SwerveDriveSimulation}. </p>
 * */
public abstract class AbstractDriveTrainSimulation extends Body {
    public static final double
            /* https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction */
            BUMPER_COEFFICIENT_OF_FRICTION = 0.65,
            /* https://simple.wikipedia.org/wiki/Coefficient_of_restitution */
            BUMPER_COEFFICIENT_OF_RESTITUTION = 0.12;

    public final DriveTrainSimulationProfile profile;

    protected AbstractDriveTrainSimulation(DriveTrainSimulationProfile profile, Pose2d initialPoseOnField) {
        this.profile = profile;

        /* width and height in world reference is flipped */
        final double WIDTH_IN_WORLD_REFERENCE = profile.length,
                HEIGHT_IN_WORLD_REFERENCE = profile.width;

        super.addFixture(
                Geometry.createRectangle(WIDTH_IN_WORLD_REFERENCE, HEIGHT_IN_WORLD_REFERENCE),
                profile.robotMass / (profile.length * profile.width),
                BUMPER_COEFFICIENT_OF_FRICTION,
                BUMPER_COEFFICIENT_OF_RESTITUTION
        );

        super.setMass(MassType.NORMAL);
        super.setLinearDamping(profile.linearVelocityDamping);
        super.setAngularDamping(profile.angularVelocityDamping);
        setSimulationWorldPose(initialPoseOnField);
    }

    /**
     * sets the robot's current pose in the simulation world
     * the robot DOES NOT drive to that pose, it teleports there instantly
     * @param robotPose the desired robot pose
     * */
    public void setSimulationWorldPose(Pose2d robotPose) {
        super.transform.set(GeometryConvertor.toDyn4jTransform(robotPose));
        super.linearVelocity.set(0, 0);
    }

    /**
     * sets the robot's speeds to a given chassis speeds
     * the robot's speeds will jump to the given speeds in a tick
     * */
    public void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
        super.setLinearVelocity(GeometryConvertor.toDyn4jLinearVelocity(givenSpeeds));
        super.setAngularVelocity(givenSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Abstract simulation periodic method.
     * Called everytime the simulation world is updated.
     * The robot should apply the propelling forces here.
     * */
    public abstract void simulationPeriodic(int iterations, double periodSeconds);

    /**
     * simulate the LINEAR friction force on the drivetrain
     * when the drivetrain is not trying to move LINEARLY
     * */
    protected void simulateChassisLinearFriction() {
        final double actualLinearVelocityPercent = getLinearVelocity().getMagnitude() / profile.maxLinearVelocity;
        final boolean robotActuallyMovingLinearly = actualLinearVelocityPercent > 0.01;
        if (robotActuallyMovingLinearly)
            /*
            * apply the friction force
            * with its direction opposite to the that of the current velocity
            * with its magnitude specified in profile
            * */
            super.applyForce(new Force(
                    super.linearVelocity
                            .getNormalized()
                            .multiply(-profile.frictionForceMagnitudeNewtons)
            ));
        else /* when the velocity is too small, just set the object at rest */
            super.setLinearVelocity(new Vector2());
    }

    /**
     * simulate the ANGULAR friction force on the drivetrain
     * when the drivetrain is not trying to move ROTATIONALLY
     * */
    protected void simulateChassisAngularFriction() {
        final double actualRotationalMotionPercent = Math.abs(getAngularVelocity() / profile.maxAngularVelocity);
        if (actualRotationalMotionPercent > 0.01)
            /*
             * apply the friction torque
             * with its sign opposite to the that of the current angular velocity
             * with its magnitude specified in profile
             * */
            super.applyTorque(Math.copySign(this.profile.angularFrictionTorqueMagnitude, -super.getAngularVelocity()));
        else /* when the velocity is too small, just set it still */
            super.setAngularVelocity(0);
    }

    public Pose2d getSimulatedDriveTrainPose() {
        return GeometryConvertor.toWpilibPose2d(getTransform());
    }

    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                getSimulatedDriveTrainPose().getRotation()
        );
    }

    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsFieldRelative() {
        return GeometryConvertor.toWpilibChassisSpeeds(getLinearVelocity(), getAngularVelocity());
    }

    public static final class DriveTrainSimulationProfile {
        public final double
                maxLinearVelocity,
                maxLinearAcceleration,
                maxAngularVelocity,
                maxAngularAcceleration,
                robotMass,
                width,
                length;

        private double
                frictionForceMagnitudeNewtons,
                linearVelocityDamping,
                angularFrictionTorqueMagnitude,
                angularVelocityDamping;

        public DriveTrainSimulationProfile(double maxLinearVelocity, double maxLinearAcceleration, double maxAngularVelocity, double maxAngularAcceleration, double robotMass, double width, double length) {
            this.maxLinearVelocity = maxLinearVelocity;
            this.maxLinearAcceleration = maxLinearAcceleration;
            this.maxAngularVelocity = maxAngularVelocity;
            this.maxAngularAcceleration = maxAngularAcceleration;
            this.robotMass = robotMass;
            this.width = width;
            this.length = length;

            final double
                    GRAVITY_CONSTANT = 9.8,
                    WHEEL_COEFFICIENT_OF_FRICTION = 0.8,
                    DRIVE_BASE_RADIUS = Math.hypot(width / 2, length/ 2);
            this.frictionForceMagnitudeNewtons = GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION * robotMass;
            this.linearVelocityDamping = maxLinearAcceleration / maxLinearVelocity;
            this.angularFrictionTorqueMagnitude = frictionForceMagnitudeNewtons * DRIVE_BASE_RADIUS;
            this.angularVelocityDamping = maxAngularAcceleration / maxAngularVelocity;
        }

        public DriveTrainSimulationProfile withFrictionForceMagnitude(double frictionForceMagnitudeNewtons) {
            this.frictionForceMagnitudeNewtons = frictionForceMagnitudeNewtons;
            return this;
        }

        public DriveTrainSimulationProfile withLinearVelocityDamping(double linearVelocityDamping) {
            this.linearVelocityDamping = linearVelocityDamping;
            return this;
        }

        public DriveTrainSimulationProfile withAngularFrictionTorqueMagnitude(double angularFrictionTorqueMagnitude) {
            this.angularFrictionTorqueMagnitude = angularFrictionTorqueMagnitude;
            return this;
        }

        public DriveTrainSimulationProfile withAngularVelocityDamping(double frictionForceMagnitudeNewtons) {
            this.frictionForceMagnitudeNewtons = frictionForceMagnitudeNewtons;
            return this;
        }

        @Override
        public String toString() {
            return String.format("RobotProfile { robotMaxVelocity=%.2f(m/s), robotMaxAcceleration=%.2f(m/s^2), robotMass=%.2f(kg), " +
                            "frictionForceMagnitude=%.2f(N), linearVelocityDamping=%.2f(N*s*m^-1), maxAngularVelocity=%.2f(rad/s), " +
                            "maxAngularAcceleration=%.2f (rad/s^2), angularDamping=%.2f(N*m*s*rad^-1), angularFrictionTorqueMagnitude=%.2f(N*m), width=%.2f(m), " +
                            "length=%.2f(m) }",
                    maxLinearVelocity, maxLinearAcceleration, robotMass, frictionForceMagnitudeNewtons, linearVelocityDamping,
                    maxAngularVelocity, maxAngularAcceleration, angularVelocityDamping, angularFrictionTorqueMagnitude, width, length);
        }
    }
}
