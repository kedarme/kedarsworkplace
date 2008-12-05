package game.util;

import game.app.Board;

/**
 * This is a utility class which is used for Comparing Array of bytes. It has
 * various function for
 * 
 * 1. Checking all elements are equal
 * 
 * 2. Deciding a Win based on number of Segments X or O has on the board
 * 
 * @author Kedar Kulkarni
 * 
 */
public class CompareArray {
	
	/**
	 * This is static method which compares whether all elements in the array
	 * are equal
	 * 
	 * @param b1
	 * @param b2
	 * @param b3
	 * @param b4
	 * @return byte
	 */
	public static byte compare(byte b1, byte b2, byte b3, byte b4) {
		/** Variable to be returned */
		byte returnVar = 0;
		// Check if all elements are equal
		if (b1 == b2 && b2 == b3 && b3 == b4) {
			// Return equal element
			returnVar = b1;
		}
		// Return zero
		return returnVar;

	}

	/**
	 * This method is used to decide a winner based on the maximum number of 3
	 * segments rule
	 * 
	 * This is used at two places.
	 * 
	 * 1. When board is full and winner has to be decided
	 * 
	 * 2. Computing the Utility function. The Utility function is defined as
	 * difference in number of 3 segment or 4 segment moves each player can
	 * make. The one with maximum moves is predicted to win the board. It is
	 * calculated by replacing all empty positions on the Board and the current
	 * players mark by 1 and replacing the other players mark by 0. Now we
	 * calculate sum of rows,columns and diagonals. Any of which sum to 3 or 4
	 * is assigned as segment to current player. This is done twice one for each
	 * player and the difference is calculated.
	 * 
	 * @param array
	 * @return byte
	 */
	public static byte decideSegmentWin(byte[] array) {
		/** Variables for number of Segments for X and O */
		int segmentsX, segmentsY = 0;
		/** Current board position */
		byte temp[] = array.clone();

		// Convert the byte Array to keep only X marks ie Player1.
		for (int i = 0; i < temp.length; i++) {
			// Mark all Player 2 indices as zero
			if (temp[i] == Board.Player2)
				temp[i] = 0; // Mark zero
			else
				temp[i] = 1; // Mark one
		}
		// Number of Segments for X
		segmentsX = countSegments(temp);

		// Current board position
		temp = array.clone();
		// Convert the byte Array to keep only O marks ie Player2.
		for (int i = 0; i < temp.length; i++) {
			// Mark all Player 1 indices as zero
			if (temp[i] == Board.Player1)
				temp[i] = 0; // Mark zero
			else
				temp[i] = 1; // Mark one
		}
		// Number of Segments for X
		segmentsY = countSegments(temp);
		// Decide who won
		if (segmentsX > segmentsY)
			return Board.Player1; // Player 1 wins
		else if (segmentsY > segmentsX)
			return Board.Player2; // Player 2 wins
		else
			return Board.TIE; // Its a tie
	}

	/**
	 * Checks all possible length wise variations of the board and returns
	 * number of segments with more 3 or more marks of the same type
	 * 
	 * @param comp
	 * @return int
	 */
	private static int countSegments(byte[] comp) {
		/** Temporary Holder */
		int[] threeResults = new int[14];
		/** The variable for number of segments */
		int sum = 0;
		// Check for row 1
		threeResults[0] = comp[0] + comp[1] + comp[2] + comp[3];
		if (threeResults[0] >= 3 && (comp[1] == comp[2])) {
			sum += 1;
		}
		// Check for row 2
		threeResults[1] = comp[4] + comp[5] + comp[6] + comp[7];
		if (threeResults[1] >= 3 && (comp[5] == comp[6])) {
			sum += 1;
		}
		// Check for row 3
		threeResults[2] = comp[8] + comp[9] + comp[10] + comp[11];
		if (threeResults[2] >= 3 && (comp[9] == comp[10])) {
			sum += 1;
		}
		// Check for row 4
		threeResults[3] = comp[12] + comp[13] + comp[14] + comp[15];
		if (threeResults[3] >= 3 && (comp[13] == comp[14])) {
			sum += 1;
		}
		// Check for column 1
		threeResults[4] = comp[0] + comp[4] + comp[8] + comp[12];
		if (threeResults[4] >= 3 && (comp[4] == comp[8])) {
			sum += 1;
		}
		// Check for column 2
		threeResults[5] = comp[1] + comp[5] + comp[9] + comp[13];
		if (threeResults[5] >= 3 && (comp[5] == comp[9])) {
			sum += 1;
		}
		// Check for column 3
		threeResults[6] = comp[2] + comp[6] + comp[10] + comp[14];
		if (threeResults[6] >= 3 && (comp[6] == comp[10])) {
			sum += 1;
		}
		// Check for column 4
		threeResults[7] = comp[3] + comp[7] + comp[11] + comp[15];
		if (threeResults[7] >= 3 && (comp[7] == comp[11])) {
			sum += 1;
		}
		// Check for diagonal across element 0 - 15
		threeResults[8] = comp[0] + comp[5] + comp[10] + comp[15];
		if (threeResults[8] >= 3 && (comp[5] == comp[10])) {
			sum += 1;
		}
		// Check for diagonal across 3 - 12
		threeResults[9] = comp[12] + comp[9] + comp[6] + comp[3];
		if (threeResults[9] >= 3 && (comp[9] == comp[6])) {
			sum += 1;
		}
		// Check for continuous elements 1 - 6 - 7
		threeResults[10] = comp[1] + comp[6] + comp[11]; // 01-23 diag
		if (threeResults[10] >= 3) { // Only 3 elements
			sum += 1;
		}
		// Check for continuous elements 4 - 9 - 14
		threeResults[11] = comp[4] + comp[9] + comp[14];
		if (threeResults[11] >= 3) { // Only 3 elements
			sum += 1;
		}
		// Check for continuous elements 8 - 5 - 2
		threeResults[12] = comp[8] + comp[5] + comp[2];
		if (threeResults[11] >= 3) { // Only 3 elements
			sum += 1;
		}
		// Check for continuous elements 13 - 10 - 7
		threeResults[13] = comp[13] + comp[10] + comp[7];
		if (threeResults[11] >= 3) { // Only 3 elements
			sum += 1;
		}
		// Return sum
		return sum;
	}

}
