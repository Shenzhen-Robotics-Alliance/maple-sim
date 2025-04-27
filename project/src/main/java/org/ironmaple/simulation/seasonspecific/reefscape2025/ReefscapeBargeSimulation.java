package org.ironmaple.simulation.seasonspecific.reefscape2025;




import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.*;

import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;

/**
 *
 *
 * <h2>Simulates the two <strong>REEF</strong>s on the field.</h2>
 *
 * <p>This class simulates the two <strong>REEF</strong>s on the field where <strong>CORAL</strong>s can be scored. It
 * includes all 12 {@link ReefscapeReefBranchesTower} instances on the field (both blue and red).
 */
public class ReefscapeBargeSimulation implements SimulatedArena.Simulatable {
    private final Arena2025Reefscape arena;
    private final List<Rectangle> barges;

    protected Rectangle redBarge = new Rectangle(1.12,  8.89);
    protected Rectangle blueBarge = new Rectangle(1.12, 8.89);
    public final double bargeBottom = 1.57;
    public final double bargeTop =  2.57;
    protected int redBargeCount=0;
    protected int blueBargeCount=0;

    public ReefscapeBargeSimulation(Arena2025Reefscape arena) {
        this.arena = arena;
        this.barges = new ArrayList<>(2);
        
        blueBarge.translate(8.805, 6.181);
        redBarge.translate(8.805, 1.884);

        

    }

    @Override
    public void simulationSubTick(int subTickNum) {
        Set<GamePieceProjectile> gamePiecesLaunched = arena.gamePieceLaunched();
        Set<GamePieceProjectile> toRemoves = new HashSet<>();
        for (Rectangle barge : barges)
            for (GamePieceProjectile gamePieceLaunched : gamePiecesLaunched)
                checkForCoralPlacement(barge, gamePieceLaunched, toRemoves);

        for (GamePieceProjectile toRemove : toRemoves) gamePiecesLaunched.remove(toRemove);
    }

    private void checkForCoralPlacement(
            Rectangle barge, GamePieceProjectile gamePieceLaunched, Set<GamePieceProjectile> toRemove) {
        if (gamePieceLaunched instanceof ReefscapeAlgaeOnFly algaeOnFly)
            if (!toRemove.contains(gamePieceLaunched) && algeaInBarge(barge, algaeOnFly))
                if (barge==redBarge){
                    redBargeCount++;
                }
                else{
                    blueBargeCount++;
                }
                toRemove.add(gamePieceLaunched);
                
    }

    protected boolean algeaInBarge(Rectangle barge, GamePieceProjectile algae){
        return barge.contains(new Vector2(algae.getPose3d().getX(), algae.getPose3d().getX()));
    }

    /**
     * Displays all the ALGAEs scored on the REEF.
     *
     * @param algaePosesToDisplay a list of {@link Pose3d} objects used to visualize the positions of the CORALs on
     *     AdvantageScope
     */
    public void addCoralsOnReefForDisplay(List<Pose3d> algaePosesToDisplay) {
        for (int i=0; i<redBargeCount; i++){
            algaePosesToDisplay.add(new Pose3d(redBarge.getCenter().x, redBarge.getCenter().y, (bargeBottom+bargeTop)/2, new Rotation3d()).plus(new Transform3d(0, i, 0, new Rotation3d())));
        }
        for (int i=0; i<blueBargeCount; i++){
            algaePosesToDisplay.add(new Pose3d(blueBarge.getCenter().x, blueBarge.getCenter().y, (bargeBottom+bargeTop)/2, new Rotation3d()).plus(new Transform3d(0, i, 0, new Rotation3d())));
        }
    }

    /** Clears all the ALGAEs scored on the REEF. */
    public void clearReef() {
        redBargeCount=0;
        blueBargeCount=0;
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
}



