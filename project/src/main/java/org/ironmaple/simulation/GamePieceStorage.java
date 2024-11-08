package org.ironmaple.simulation;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.ironmaple.simulation.GamePiece.GamePieceVariant;
import org.ironmaple.utils.RuntimeLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class GamePieceStorage {
    private final Supplier<Pose3d> localizer;
    private final GamePiece[] pieces;
    private final Transform3d[] pieceTransforms;

    /**
     * Creates a new GamePieceStorage that can store the given number of pieces.
     * 
     * @param numPieces the number of pieces to store
     */
    public GamePieceStorage(Supplier<Pose3d> localizer, int numPieces) {
        pieceTransforms = new Transform3d[numPieces];
        for (int i = 0; i < numPieces; i++) {
            pieceTransforms[i] = new Transform3d();
        }
        pieces = new GamePiece[numPieces];
        this.localizer = localizer;
    }

    /**
     * Creates a new GamePieceStorage that can store as many pieces
     * as there are transforms.
     * 
     * <p> This will allow the simulation to publish the exact coordinates
     * of the game pieces for you.
     * 
     * @param firstTransform the first transform
     * @param otherTransforms the other transforms
     */
    public GamePieceStorage(Supplier<Pose3d> localizer, Transform3d firstTransform, Transform3d... otherTransforms) {
        // This doesn't just use only varags so you can't accidentally make a storage
        // with a capacity of 0.
        pieceTransforms = new Transform3d[otherTransforms.length + 1];
        pieceTransforms[0] = firstTransform;
        for (int i = 0; i < otherTransforms.length; i++) {
            pieceTransforms[i + 1] = otherTransforms[i];
        }
        pieces = new GamePiece[pieceTransforms.length];
        this.localizer = localizer;
    }

    /**
     * Returns the capacity of this storage.
     * 
     * @return the capacity of this storage
     */
    public int capacity() {
        return pieceTransforms.length;
    }

    /**
     * Returns the number of {@link GamePiece}s currently stored in this storage.
     * 
     * @return the number of {@link GamePiece}s currently stored in this storage
     */
    public int stored() {
        int count = 0;
        for (GamePiece piece : pieces) {
            if (piece != null) {
                count++;
            }
        }
        return count;
    }

    /**
     * Moves all {@link GamePiece}s up an index if the spot is available.
     */
    public void scoochUp() {
        for (int i = pieces.length - 1; i > 0; i--) {
            if (pieces[i] == null && pieces[i - 1] != null) {
                pieces[i] = pieces[i - 1];
                pieces[i - 1] = null;
            }
        }
    }

    /**
     * Moves all {@link GamePiece}s down an index if the spot is available.
     */
    public void scoochDown() {
        for (int i = 0; i < pieces.length - 1; i++) {
            if (pieces[i] == null && pieces[i + 1] != null) {
                pieces[i] = pieces[i + 1];
                pieces[i + 1] = null;
            }
        }
    }

    private int getIndexOf(GamePiece piece) {
        for (int i = 0; i < pieces.length; i++) {
            if (pieces[i] == piece) {
                return i;
            }
        }
        return -1;
    }

    private void handleNewGamepiece(GamePiece piece) {
        Supplier<Transform3d> transformSupplier = () -> {
            int index = getIndexOf(piece);
            if (index == -1) {
                return new Transform3d();
            }
            return pieceTransforms[index];
        };
        piece.releaseControl().intakeSudo(localizer, transformSupplier);
        RuntimeLog.debug("GamePieceStorage: New game piece added to storage");
    }

    /**
     * Adds a {@link GamePiece} to the highest available slot in the storage.
     * 
     * @param piece the {@link GamePiece} to add
     * @return true if the piece was added, false if the storage is full
     */
    public boolean addHighestAvailable(GamePiece piece) {
        for (int i = pieces.length - 1; i >= 0; i--) {
            if (pieces[i] == null) {
                pieces[i] = piece;
                handleNewGamepiece(piece);
                return true;
            }
        }
        return false;
    }

    /**
     * Adds a {@link GamePiece} to the lowest available slot in the storage.
     * 
     * @param piece the {@link GamePiece} to add
     * @return true if the piece was added, false if the storage is full
     */
    public boolean addLowestAvailable(GamePiece piece) {
        for (int i = 0; i < pieces.length; i++) {
            if (pieces[i] == null) {
                pieces[i] = piece;
                handleNewGamepiece(piece);
                return true;
            }
        }
        return false;
    }

    /**
     * Adds a {@link GamePiece} to the storage at the lowest index.
     * This will move other pieces up to make room.
     * 
     * @param piece the {@link GamePiece} to add
     * @return
     */
    public boolean addLowestDisplace(GamePiece piece) {
        if (this.stored() == this.capacity()) {
            return false;
        }
        if (pieces[0] != null) {
            scoochUp();
        }
        pieces[0] = piece;
        handleNewGamepiece(piece);
        return true;
    }

    /**
     * Adds a {@link GamePiece} to the storage at the highest index.
     * This will move other pieces down to make room.
     * 
     * @param piece the {@link GamePiece} to add
     * @return true if the piece was added, false if the storage is full
     */
    public boolean addHighestDisplace(GamePiece piece) {
        if (this.stored() == this.capacity()) {
            return false;
        }
        if (pieces[pieces.length - 1] != null) {
            scoochDown();
        }
        pieces[pieces.length - 1] = piece;
        handleNewGamepiece(piece);
        return true;
    }

    /**
     * Adds a {@link GamePiece} to the storage at a random index.
     * 
     * @param piece the {@link GamePiece} to add
     * @return true if the piece was added, false if the storage is full
     */
    public boolean addRandom(GamePiece piece) {
        ArrayList<Integer> emptyIndices = new ArrayList<>();
        for (int i = 0; i < pieces.length; i++) {
            if (pieces[i] == null) {
                emptyIndices.add(i);
            }
        }
        if (emptyIndices.isEmpty()) {
            return false;
        }
        pieces[emptyIndices.get((int) (Math.random() * emptyIndices.size()))] = piece;
        handleNewGamepiece(piece);
        return true;
    }

    /**
     * Allows checking the variant of a {@link GamePiece} at a specific index.
     * 
     * @param index the index to check
     * @return the variant of the {@link GamePiece} at the index, or empty if there is no piece there
     */
    public Optional<GamePieceVariant> peekAt(int index) {
        return Optional.ofNullable(pieces[index]).map(GamePiece::variant);
    }

    /**
     * Pulls a {@link GamePiece} from the top of the storage.
     * 
     * <p> The {@link GamePiece} from this will be {@code UserControlled}.
     * 
     * @param scooch whether to move the other pieces up to fill the gap
     * @return the {@link GamePiece} that was pulled, or empty if the storage is empty
     */
    public Optional<GamePiece> pullHighest(boolean scooch) {
        final int idx = pieces.length - 1;
        if (pieces[idx] == null) {
            return Optional.empty();
        }
        GamePiece piece = pieces[idx];
        pieces[idx] = null;
        if (scooch) {
            scoochUp();
        }
        RuntimeLog.debug("GamePieceStorage: Game piece removed from storage");
        return Optional.of(piece.userControlled());
    }

    /**
     * Pulls a {@link GamePiece} from the bottom of the storage.
     * 
     * <p> The {@link GamePiece} from this will be {@code UserControlled}.
     * 
     * @param scooch whether to move the other pieces down to fill the gap
     * @return the {@link GamePiece} that was pulled, or empty if the storage is empty
     */
    public Optional<GamePiece> pullLowest(boolean scooch) {
        final int idx = 0;
        if (pieces[idx] == null) {
            return Optional.empty();
        }
        GamePiece piece = pieces[idx];
        pieces[idx] = null;
        if (scooch) {
            scoochDown();
        }
        RuntimeLog.debug("GamePieceStorage: Game piece removed from storage");
        return Optional.of(piece.userControlled());
    }
}
