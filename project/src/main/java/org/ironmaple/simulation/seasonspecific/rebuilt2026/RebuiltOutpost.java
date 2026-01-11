package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.*;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

/**
 *
 *
 * <h2>Simulates a <strong>PROCESSOR</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>PROCESSOR</strong>s on the field where <strong>ALGAE</strong>s can be scored. It
 * will automatically launch the algae scored into the apposing barge and always hit.
 */
public class RebuiltOutpost extends Goal {

    protected static final Translation3d blueHPPose = new Translation3d(6.34, -0.5, 0);
    protected static final Translation3d redHPPose = new Translation3d(8.069326, 16.794988, 0);
    protected static final Translation3d blueLaunchPose = new Translation3d(6.34, 0, 0);
    protected static final Translation3d redLaunchPose = new Translation3d(11.5, 8, 0);

    StructPublisher<Pose3d> posePublisher;

    protected int fuelCount=24;

    /**
     *
     *
     * <h2>Creates an processor of the specified color.</h2>
     *
     * @param arena The host arena of this processor.
     * @param isBlue Wether this is the blue processor or the red one.
     */
    public RebuiltOutpost(Arena2025Reefscape arena, boolean isBlue) {
        super(
                arena,
                Inches.of(0),
                Centimeters.of(0),
                Centimeters.of(0),
                "Fuel",
                isBlue ? blueProcessorPose : redProcessorPose,
                isBlue);

        StructPublisher<Pose3d> heldAlgaePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueOutpost" : "RedOutpost", Pose3d.struct)
                .publish();
        heldAlgaePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "AlgaeInProcessor", 1);
        this.fuelCount++;


    }

    @Override
    public void draw(List<Pose3d> drawList) {
        return;
    }

    public void reset(){
        fuelCount=24;
    }

    public void throwForGoal(){
        throwAtPoint(null, pieceAngleTolerance);
    }

    public void throwAtPoint(Rotation2d yaw, Angle pitch){
        if (isBlue) {
            this.arena.addGamePieceProjectile(new RebuiltFuelOnFly(
                    BlueLaunchPose.toTranslation2d(),
                    new Translation2d(),
                    new ChassisSpeeds(),
                    yaw,
                    Meters.of(1.7),
                    MetersPerSecond.of(7),
                    pitch));
        } else {
            this.arena.addGamePieceProjectile(new RebuiltFuelOnFly(
                    redLaunchPose.toTranslation2d(),
                    new Translation2d(),
                    new ChassisSpeeds(),
                    yaw,
                    Meters.of(1.7),
                    MetersPerSecond.of(7),
                    pitch));
        }
    }
}
