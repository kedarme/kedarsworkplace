/**
 * 
 */
package game.app;

import game.util.CompareArray;

import java.util.ArrayList;
import java.util.List;

/**
 * This class defines the 4x4 Tic Tac Toe board. It also has methods to evaluate
 * if the game is over and if so returns the winner.
 * 
 * @author Kedar Kulkarni
 * 
 */
public class Board {
	/** Constant defining a Tie */
	public static final byte TIE = 101;
	/** Constant defining a Empty */
	public static final byte EMPTY = 0;
	/** Constant defining a Player1 */
	public static final byte Player1 = -1;
	/** Constant defining a PLayer2 */
	public static final byte Player2 = 1;
	/** The machines copy of the board positions */
	private byte board[];
	/** The winner */
	private byte winner;

	/**
	 * The constructor for the board. This sets all positions to Empty
	 */
	public Board() {
		super();
		// Create a new board. Java Initializes to zero which is equivalent to
		// the value of the Empty position
		this.board = new byte[16];
	}

	// Returns the current board state
	public byte[] getBoard() {
		return board;
	}

	// Set board the board to state passed
	public void setBoard(byte[] ar) {
		this.board = ar;
	}

	/**
	 * Returns All the Legal Moves remaining on the board. The values are
	 * returned as a List containing array of empty positions.
	 * 
	 * @return List<Integer>
	 */
	public List<Integer> getLegalMoves() {
		// Create a Object of List type
		List<Integer> returnList = new ArrayList<Integer>();
		// Loop through the whole board
		for (int index = 0; index <= 15; index++) {
			// Check if the current position is empty
			if (this.board[index] == Board.EMPTY) {
				// Add the position to the list if position is empty
				returnList.add(index);
			}
		}
		// Return the positions
		return returnList;
	}

	/**
	 * Method updates a position on the board.
	 * 
	 * @param row
	 * @param column
	 * @param value
	 */
	public void updateBoard(Integer index, byte value) {
		// Update the index with the value passed
		this.board[index] = value;
	}

	/**
	 * Check if the game is complete. The game would be complete if we have a
	 * definite winner or if all positions on the board are filled
	 * 
	 * @return
	 */
	public Boolean isOver() {
		// Call the isWinner subroutine. This sets the winner variable depending
		// on the condition of the board
		this.isWinner();
		// Check the value of the winner variable
		if (this.winner == Board.TIE) {
			// Game is Over with a Tie
			return true;
		} else if (this.winner == Board.EMPTY) {
			// Game is not complete
			return false;
		}
		// We have a definite winner
		return true;
	}

	/**
	 * Determines if we have a winning player on the boars
	 * 
	 */
	public void isWinner() {
		/** Variable which checks if any position on the board is Empty */
		boolean isFull = true;
		// Loop through the board looking for Empty positions
		for (int index = 0; index <= 15; index++) {
			// Check if the position is Empty
			if (this.board[index] == Board.EMPTY) {
				// Change the variable to true
				isFull = false;
				// Break out once we find one empty position
				break;
			}
		}
		// Check for row 1
		this.winner = CompareArray.compare(this.board[0], this.board[1],
				this.board[2], this.board[3]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for row 2
		this.winner = CompareArray.compare(this.board[4], this.board[5],
				this.board[6], this.board[7]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for row 3
		this.winner = CompareArray.compare(this.board[8], this.board[9],
				this.board[10], this.board[11]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for row 4
		this.winner = CompareArray.compare(this.board[12], this.board[13],
				this.board[14], this.board[15]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for col 1
		this.winner = CompareArray.compare(this.board[0], this.board[4],
				this.board[8], this.board[12]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for col 2
		this.winner = CompareArray.compare(this.board[1], this.board[5],
				this.board[9], this.board[13]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for col 3
		this.winner = CompareArray.compare(this.board[2], this.board[6],
				this.board[10], this.board[14]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for col 4
		this.winner = CompareArray.compare(this.board[3], this.board[7],
				this.board[11], this.board[15]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for diag 1
		this.winner = CompareArray.compare(this.board[0], this.board[5],
				this.board[10], this.board[15]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// Check for diag 2
		this.winner = CompareArray.compare(this.board[12], this.board[9],
				this.board[6], this.board[3]);
		if (this.winner == Board.Player1 || this.winner == Board.Player2)
			return;
		// The CompareArray.compare method only returns a definite winner if
		// any. If we don't have a definite winner and the board is full
		// check for the maximum 3 segment rule for a winner
		if (this.winner == Board.EMPTY && isFull) {
			// Call the Corresponding Method for 3 segment win. This will return
			// a Tie if the number of 3 segments is equal for both X and O
			this.winner = CompareArray.decideSegmentWin(this.board);
		}
	}

	public byte getWinner() {
		// We want only the winner or tie
		byte returnVal = (this.winner == Board.TIE) ? 0 : this.winner;
		return returnVal;
	}

	public String getWinnerString() {
		String stringWinner;
		stringWinner = (this.winner == Board.Player1) ? "X wins."
				: (this.winner == Board.Player2) ? "O wins." : "It was a Tie.";
		return stringWinner;
	}
}
