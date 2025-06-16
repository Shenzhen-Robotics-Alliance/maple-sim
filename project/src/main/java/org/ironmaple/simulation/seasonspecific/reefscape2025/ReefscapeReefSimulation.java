package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.goal;

class ReefscapeReefSimulation implements SimulatedArena.Simulatable {
    protected final List<ReefscapeReefBranch> branches;
    public ReefscapeAlgaeOnField algae;
    private StructArrayPublisher<Pose3d> reefPub;
    Pose3d[] branchPoses;

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
        SmartDashboard.putString("testFromReef", "yay");

        reefPub.set(branchPoses);
    }

    public void draw(List<Pose3d> coralPosesToDisplay) {
        for (ReefscapeReefBranch branch : branches) {
            branch.draw(coralPosesToDisplay);
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        // System.out.println("test");
        for (ReefscapeReefBranch branch : branches) {
            // System.out.println("New branch");
            // System.out.println(branch.level);
            // System.out.println(branch.col);
            branch.simulationSubTick(subTickNum);
        }
    }

    public void clearReef() {
        for (ReefscapeReefBranch branch : branches) {
            branch.clear();
        }
    }
}
