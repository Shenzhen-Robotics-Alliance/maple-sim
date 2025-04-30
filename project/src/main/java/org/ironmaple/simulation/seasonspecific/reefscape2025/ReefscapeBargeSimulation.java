
package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Centimeters;

import java.util.*;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.goal;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;


/**
 *
 *
 * <h2>Simulates the two <strong>REEF</strong>s on the field.</h2>
 *
 * <p>This class simulates the two <strong>REEF</strong>s on the field where <strong>CORAL</strong>s can be scored. It
 * includes all 12 {@link ReefscapeReefBranchesTower} instances on the field (both blue and red).
 */
public class ReefscapeBargeSimulation extends goal {


    protected static final Translation3d blueBargePose = new Translation3d(8.805, 6.181, 1.57);
    protected static final Translation3d redBargePose = new Translation3d(8.805, 1.884, 1.57);

    public ReefscapeBargeSimulation(Arena2025Reefscape arena, boolean isBlue) {
        super(arena, Centimeters.of(112), Centimeters.of(8.89), Centimeters.of(100), ReefscapeAlgaeOnFly.class, isBlue? blueBargePose: redBargePose, isBlue);     
    }

    /**
     * Displays all the ALGAEs scored on the REEF.
     *
     * @param algaePosesToDisplay a list of {@link Pose3d} objects used to visualize the positions of the CORALs on
     *     AdvantageScope
     */
    public void draw(List<Pose3d> algaePosesToDisplay) {
        for (int i = 0; i < gamePieceCount; i++) {
            algaePosesToDisplay.add(new Pose3d(
                            xyBox.getCenter().x,
                            xyBox.getCenter().y,
                            elevation.in(Units.Meters) + height.in(Units.Meters) / 2,
                            new Rotation3d())
                    .plus(new Transform3d(0, i, 0, new Rotation3d())));
        }

    }

    /**
     * Returns an optional instance.
     *
     * @return (optionally) an instance of this class, empty if
     */
    public static Optional<ReefscapeReefSimulation> getInstance() {
        if (SimulatedArena.getInstance() instanceof Arena2025Reefscape arena2025Reefscape)
            return Optional.of(arena2025Reefscape.reefSimulation);
        return Optional.empty();
    }

    @SuppressWarnings("static-access")
    @Override
    protected void addPoints() {
        if (isBlue) arena.addToBlueScore(4);
        
        else arena.addToRedScore(4);
        
    }
}

 