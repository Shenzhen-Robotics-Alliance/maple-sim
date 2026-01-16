package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.SimulatedArena;

/**
 *
 *
 * <h2>Simulates a <strong>REEF</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>REEF</strong>s on the field where <strong>CORAL</strong>s can be scored. This class
 * does not directly handle scoring uses an array of {@link ReefscapeReefBranch} objects. However for all other purposes
 * this class behaves and can be used like a normal goal.
 */
public class ReefscapeReefSimulation implements SimulatedArena.Simulatable {
    protected final List<ReefscapeReefBranch> branches;
    public ReefscapeAlgaeOnField algae;
    private StructArrayPublisher<Pose3d> reefPub;
    Pose3d[] branchPoses;

    /**
     *
     *
     * <h2>Creates an reef of the specified color.</h2>
     *
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
                        new Pose3d(tempPose.getTranslation(), Goal.flipRotation(tempPose.getRotation()));
            }
        }
        reefPub = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SmartDashboard/MapleSim/" + (isBlue ? "BlueReef" : "RedReef"), Pose3d.struct)
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

    /**
     *
     *
     * <h2>Resets the reef to its original state.</h2>
     */
    public void clearReef() {
        for (ReefscapeReefBranch branch : branches) {
            branch.clear();
        }
    }

    /**
     * Obtains the amount of <strong>CORAL</strong> held on the <strong>BRANCHES</strong>.
     *
     * <p>This method returns a 2D array of size 12 x 4, where each entry represents the number of
     * <strong>CORAL</strong>s held on a particular branch.
     *
     * <p>The <strong>BRANCHES</strong> are tracked in FMS as A, B, C, D, E, F, G, H, I, J, K, L (as per the game
     * manual), and are mapped to indices 0, 1, 2, ... in the array.
     *
     * <p>The [i][j] entry in the array represents the number of <strong>CORAL</strong>(s) held on the L<code>j-1</code>
     * branch in the <code>i</code>th section.
     *
     * <p>For example, <code>getBranches()[2][3]</code> returns the number of CORALs held on L4 of Branch C.
     *
     * <p>Note that L2, L3, and L4 can only hold one <strong>CORAL</strong>, while L1 can hold up to two
     * <strong>CORAL</strong>s.
     *
     * @return a 2D array where each entry represents the number of <strong>CORAL</strong> held on each branch
     */
    public int[][] getBranches() {
        int[][] toReturn = new int[12][4];
        for (ReefscapeReefBranch branch : branches) {
            toReturn[branch.column][branch.level] = branch.getGamePieceCount();
        }
        return toReturn;
    }
}
