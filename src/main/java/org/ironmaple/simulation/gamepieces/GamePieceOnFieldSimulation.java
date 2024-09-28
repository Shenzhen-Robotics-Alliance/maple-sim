package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import java.util.function.DoubleSupplier;

/**
 * Simulates a game piece on the field.
 * Game pieces HAVE collision spaces.
 * They can also be "grabbed" by an {@link IntakeSimulation}.
 * */
public class GamePieceOnFieldSimulation extends Body {
    public static final double
            LINEAR_DAMPING = 3.5,
            ANGULAR_DAMPING = 5,
            COEFFICIENT_OF_FRICTION = 0.8,
            COEFFICIENT_OF_RESTITUTION = 0.3,
            MINIMUM_BOUNCING_VELOCITY = 0.2;

    private final DoubleSupplier heightSupplier;
    public final String type;

    public GamePieceOnFieldSimulation(String type, Convex shape, double gamePieceHeight, double mass, Translation2d initialPosition) {
        this(type, shape, () -> gamePieceHeight / 2, mass, initialPosition, new Translation2d());
    }

    public GamePieceOnFieldSimulation(String type, Convex shape, DoubleSupplier heightSupplier, double mass, Translation2d initialPosition, Translation2d initialVelocityMPS) {
        super();
        this.type = type;
        this.heightSupplier = heightSupplier;

        BodyFixture bodyFixture = super.addFixture(shape);

        bodyFixture.setFriction(COEFFICIENT_OF_FRICTION);
        bodyFixture.setRestitution(COEFFICIENT_OF_RESTITUTION);
        bodyFixture.setRestitutionVelocity(MINIMUM_BOUNCING_VELOCITY);

        bodyFixture.setDensity(mass / shape.getArea());
        super.setMass(MassType.NORMAL);

        super.translate(GeometryConvertor.toDyn4jVector2(initialPosition));

        super.setLinearDamping(LINEAR_DAMPING);
        super.setAngularDamping(ANGULAR_DAMPING);
        super.setBullet(true);

        super.setLinearVelocity(GeometryConvertor.toDyn4jVector2(initialVelocityMPS));
    }

    /**
     * sets the velocity of this game piece
     * change will apply immediately
     * */
    public void setVelocity(ChassisSpeeds chassisSpeedsWorldFrame) {
        super.setLinearVelocity(
                GeometryConvertor.toDyn4jLinearVelocity(chassisSpeedsWorldFrame)
        );
        super.setAngularVelocity(chassisSpeedsWorldFrame.omegaRadiansPerSecond);
    }

    public Pose2d getPoseOnField() {
        return GeometryConvertor.toWpilibPose2d(super.getTransform());
    }

    public Pose3d getPose3d() {
        final Pose2d pose2d = getPoseOnField();
        return new Pose3d(
                pose2d.getX(), pose2d.getY(),
                heightSupplier.getAsDouble(),
                new Rotation3d(
                        0, 0,
                        pose2d.getRotation().getRadians()
                )
        );
    }
}
