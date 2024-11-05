package org.ironmaple.utils.mathutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.geometry.Rotation;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;

/** utils to convert between WPILIB and dyn4j geometry classes */
public class GeometryConvertor {
    public static Vector2 toDyn4jVector2(Translation2d wpilibTranslation2d) {
        return new Vector2(wpilibTranslation2d.getX(), wpilibTranslation2d.getY());
    }

    public static Translation2d toWpilibTranslation2d(Vector2 dyn4jVector2) {
        return new Translation2d(dyn4jVector2.x, dyn4jVector2.y);
    }

    public static Rotation toDyn4jRotation(Rotation2d wpilibRotation2d) {
        return new Rotation(wpilibRotation2d.getRadians());
    }

    public static Rotation2d toWpilibRotation2d(Rotation dyn4jRotation) {
        return new Rotation2d(dyn4jRotation.toRadians());
    }

    public static Transform toDyn4jTransform(Pose2d wpilibPose2d) {
        final Transform transform = new Transform();
        transform.setTranslation(toDyn4jVector2(wpilibPose2d.getTranslation()));
        transform.setRotation(toDyn4jRotation(wpilibPose2d.getRotation()));
        return transform;
    }

    public static Pose2d toWpilibPose2d(Transform dyn4jTransform) {
        return new Pose2d(
                toWpilibTranslation2d(dyn4jTransform.getTranslation()),
                toWpilibRotation2d(dyn4jTransform.getRotation()));
    }

    public static Vector2 toDyn4jLinearVelocity(ChassisSpeeds wpilibChassisSpeeds) {
        return new Vector2(wpilibChassisSpeeds.vxMetersPerSecond, wpilibChassisSpeeds.vyMetersPerSecond);
    }

    public static ChassisSpeeds toWpilibChassisSpeeds(Vector2 dyn4jLinearVelocity, double angularVelocityRadPerSec) {
        return new ChassisSpeeds(dyn4jLinearVelocity.x, dyn4jLinearVelocity.y, angularVelocityRadPerSec);
    }

    public static Translation2d getChassisSpeedsTranslationalComponent(ChassisSpeeds chassisSpeeds) {
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }
}
