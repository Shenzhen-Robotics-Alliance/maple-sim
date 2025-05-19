package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

class ReefscapeReefSimulation implements SimulatedArena.Simulatable {
    protected final List<ReefscapeReefBranch> branches;
    public ReefscapeAlgaeOnField algae;

    ReefscapeReefSimulation(Arena2025Reefscape arena, boolean isBlue) {
        branches = new ArrayList<ReefscapeReefBranch>(48);
        for (int tower = 0; tower < 12; tower++) {
            for (int level = 0; level < 4; level++) {
                branches.add(new ReefscapeReefBranch(arena, isBlue, level, tower));
            }
        }
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

    public void clearReef() {
        for (ReefscapeReefBranch branch : branches) {
            branch.clear();
        }
    }
}
