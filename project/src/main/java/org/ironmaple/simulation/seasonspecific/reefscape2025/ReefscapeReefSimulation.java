package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.goal;


/**
 *
 *
 * <h2>Simulates a <strong>REEF</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>REEF</strong>s on the field where <strong>CORAL</strong>s can be scored.
 * This class does not directly handle scoring uses an array of {@link ReefscapeReefBranch} objects. 
 * However for all other purposes this class behaves and can be used like a normal goal.
 */
class ReefscapeReefSimulation implements SimulatedArena.Simulatable {
    protected final List<ReefscapeReefBranch> branches;
    public ReefscapeAlgaeOnField algae;
    private StructArrayPublisher<Pose3d> reefPub;
    Pose3d[] branchPoses;


    /**
     * <h2>Creates an reef of the specified color.</h2>
     * @param arena The host arena of this reef.
     * @param isBlue Wether this is the blue reef or the red one.
     */
    ReefscapeReefSimulation(Arena2025Reefscape arena, boolean isBlue) {
        branches = new ArrayList<ReefscapeReefBranch>(48);
        branchPoses = new Pose3d[96];
        for (int tower = 0; tower < 12; tower++) {
            for (int level = 0; level < 4; level++) {
                branches.add(new ReefscapeReefBranch(arena, isBlue, level, tower));
                Pose3d tempPose = branches.get(branches.size() - 1).getPose();

                branchPoses[2 * (tower * 4 + level)] = tempPose;

                branchPoses[2 * (tower * 4 + level) + 1] =
                        new Pose3d(tempPose.getTranslation(), goal.flipRotation(tempPose.getRotation()));
            }
        }
        reefPub = NetworkTableInstance.getDefault()
                .getStructArrayTopic(isBlue ? "BlueReef" : "RedReef", Pose3d.struct)
                .publish();

        reefPub.set(branchPoses);
    }

    public void draw(List<Pose3d> coralPosesToDisplay) {
        for (ReefscapeReefBranch branch : branches) {
            branch.draw(coralPosesToDisplay);
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        for (ReefscapeReefBranch branch : branches) {
            branch.simulationSubTick(subTickNum);
        }
    }

    /**<h2>Resets the reef to its original state.</h2>*/
    public void clearReef() {
        for (ReefscapeReefBranch branch : branches) {
            branch.clear();
        }
    }
}
