package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Convex;

import java.util.function.DoubleSupplier;

public class GamePieceOnFieldSimulation {
	/**
	 * @deprecated Use {@link GamePieceOnField} instead.
	 */
	@Deprecated(forRemoval = true)
	public GamePieceOnFieldSimulation(String type, Convex shape, double gamePieceHeight, double mass, Translation2d initialPosition) {}
	
	/**
	 * @deprecated Use {@link GamePieceOnField} instead.
	 */
	@Deprecated(forRemoval = true)
	public GamePieceOnFieldSimulation(
		String type,
		Convex shape,
		DoubleSupplier zPositionSupplier,
		double mass,
		Translation2d initialPosition,
		Translation2d initialVelocityMPS
	){}
}
