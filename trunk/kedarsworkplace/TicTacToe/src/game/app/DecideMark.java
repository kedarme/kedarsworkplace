package game.app;

import game.util.CompareArray;

import java.util.Iterator;

/**
 * 
 * This class implements Alpha-Beta-Search Algorithm to decide which position
 * the machine will mark as the next move.
 * 
 * @author Kedar Kulkarni
 * 
 */
public class DecideMark {
	/** The tree search depth */
	Integer depth = 0;
	/** Indicates the next Player */
	Integer markNextMove;

	/**
	 * This is the method exposed to external classes. Depending on which Player
	 * is making the next move it calls the min or max position. X => Min O =>
	 * Max
	 * 
	 * @param b
	 * @param nextMark
	 * @return Integer
	 */
	public Integer returnMove(Board b, byte nextMark) {
		// Decide which function to call
		if (nextMark == Board.Player1) {
			// Call Min function for Player1 that is we are marking X
			AlphaBeta(b, true, Board.Player1, Board.Player2);
		} else {
			// Call Max Function for Player 2 that is we are marking O
			AlphaBeta(b, false, Board.Player1, Board.Player2);
		}
		// Return the next move
		return markNextMove;
	}

	/**
	 * The Min-Max function. The toggle between min or max function happens
	 * based on the minFunc variable
	 * 
	 * @param b
	 * @param minFunc
	 * @param Alpha
	 * @param Beta
	 * @return byte
	 */
	private byte AlphaBeta(Board b, boolean minFunc, byte Alpha, byte Beta) {
		// Terminal Tests
		// Checks if the game is Over
		boolean isOver = b.isOver();
		// Terminate if the maximum defined state has been reached or Terminal
		// state is reached
		if (depth >= Play.depth || isOver) {
			if (isOver) { // If Terminal State is reached
				// Return the winner
				return b.getWinner();
			} else { // If maximum depth was reached
				// Return Utility function value which calculates which player
				// has maximum chances of win
				return CompareArray.decideSegmentWin(b.getBoard());
			}
		}
		// The main min-max function
		if (minFunc) { // Process the min function for Player 1
			// Set value v to the win value for Player 2
			int v = Board.Player2;
			// Get all available moves
			Iterator<Integer> it = b.getLegalMoves().iterator();
			// Loop through all available moves
			while (it.hasNext()) {
				// Get the next move
				Integer next = it.next();
				// Create a local copy of the board
				Board localB = new Board();
				// Copy the state of passed board to the local copy
				localB.setBoard(b.getBoard().clone());
				// Update the local copy with the next available move
				localB.updateBoard(next, Board.Player1);
				// Increase the depth as we look at the child
				depth++;
				// Call the min function with the current board state
				int maximumValueOfSuccessor = AlphaBeta(localB, false, Alpha,
						Beta);
				// Decrease the depth when we return from the child node
				depth--;
				// Check if Player2 is loosing
				if (maximumValueOfSuccessor < v) {
					// Update with the maximum value
					v = maximumValueOfSuccessor;
					// Update the variable which will indicate the next move
					// that the machine will made
					this.markNextMove = next;
				}
				// Check that Alpha can be pruned
				if (v <= Alpha) {
					// Pruning the current branch
					return (byte) v;
				}
				// Set Beta to be v if its less than Beta
				if (v < Beta) {
					Beta = (byte) v;
				}
			}
			// Default return the current node value
			return (byte) v;

		} else { // Process the max function for Player 2
			// Set value v to the win value for Player 1
			int v = Board.Player1;
			// Get all available moves
			Iterator<Integer> it = b.getLegalMoves().iterator();
			// Loop through all available moves
			while (it.hasNext()) {
				// Get the next move
				Integer next = it.next();
				// Create a local copy of the board
				Board localB = new Board();
				// Copy the state of passed board to the local copy
				localB.setBoard(b.getBoard().clone());
				// Update the local copy with the next available move
				localB.updateBoard(next, Board.Player2);
				// Increase the depth as we look at the child
				depth++;
				// Call the min function with the current board state
				int minimumValueOfSuccessor = AlphaBeta(localB, true, Alpha,
						Beta);
				// Decrease the depth when we return from the child node
				depth--;
				// Check if Player 1 is loosing
				if (minimumValueOfSuccessor > v) {
					// Update with the maximum value
					v = minimumValueOfSuccessor;
					// Update the variable which will indicate the next move
					// that the machine will made
					this.markNextMove = next;
				}
				// Check that Beta can be pruned
				if (v >= Beta) {
					// Pruning the current branch
					return (byte) v;
				}
				// Set Alpha to be v if its more than Alpha
				if (v > Alpha) {
					Alpha = (byte) v;
				}
			}
			// Default return the current node value
			return (byte) v;
		}
	}
}
