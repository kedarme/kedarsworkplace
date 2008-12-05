package game.app;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 * This is the main class and entry point for the program. It also creates the
 * UI. The UI has 1. Options to select difficulty levels from
 * Beginner,Intermediate,Expert 2. Reset the board. 3. Decide whether player or
 * computer makes the first move.
 * 
 * @author Kedar Kulkarni
 * 
 */
public class Play extends JFrame {

	/** The Board */
	Board b = new Board();
	/** Default UID for JFrame */
	private static final long serialVersionUID = 1L;
	/** Depth Level for Expert Difficulty */
	private static final int Expert = 9;
	/** Depth Level for Expert Difficulty */
	private static final int Intermediate = 7;
	/** Depth Level for Expert Difficulty */
	private static final int Beginner = 6;

	/**
	 * First Mark is Player1. This variable decides what mark to show on the
	 * board
	 */
	byte nextMark = Board.Player1;
	/** Sets the machines turn */
	boolean machinePlays = false;
	/** Cut off level */
	static int depth = Play.Expert;
	/** 4x4 Tic-Tac-Toe Grid */
	JButton buttons[] = new JButton[16];
	/** Buttons for the difficulty levels */
	JButton levels[] = new JButton[3];
	/** Refresh/ Reset Button */
	JButton retry = new JButton("Refresh");
	/** String for Info Message Label */
	String newGameMssg = "<html>Starting a new game. Currently palying as \"Expert\". Change Difficulty Level below. To begin game click on the grid or choose that the computer makes the first move. </html>";
	/** Info Label */
	JLabel label = new JLabel(newGameMssg);
	/** Button for making the computer play first turn */
	JButton compFirst = new JButton("Yes");

	/**
	 * This is the constructor for this class. It builds the UI, calls the
	 * listener to decipher the click and puts the X,O marks on the board.s
	 */
	public Play() {
		// UI for the board
		JPanel jpCenter = new JPanel(new GridLayout(4, 4));
		for (int i = 0; i < 16; i++) {
			// Set up individual blocks of the grid
			buttons[i] = new JButton();
			// Set Color
			buttons[i].setBackground(Color.WHITE);
			// Add reference to class which would process the click
			buttons[i].addActionListener(new ListenMove(i));
			// Add button on the grid
			jpCenter.add(buttons[i]);
		}
		// Set up the upper portion of the UI
		JPanel jpNorth = new JPanel(new BorderLayout());
		// Set up the level selecting buttons
		JPanel levelIndicators = new JPanel();
		// Makes the Label
		levelIndicators.add(new JLabel("Select Difficulty Level"));
		// Button for Beginner
		levels[0] = new JButton("Beginner");
		// Add reference to class which would process the click
		levels[0].addActionListener(new ListenMove(-1));
		// Add button on the grid
		levelIndicators.add(levels[0]);
		// Button for Intermediate
		levels[1] = new JButton("Intermediate");
		// Add reference to class which would process the click
		levels[1].addActionListener(new ListenMove(-2));
		// Add button on the grid
		levelIndicators.add(levels[1]);
		// Button for Expert
		levels[2] = new JButton("Expert");
		// Add reference to class which would process the click
		levels[2].addActionListener(new ListenMove(-3));
		// Add button on the grid
		levelIndicators.add(levels[2]);
		// Enable Default Level during initial setup
		levels[2].setEnabled(false);
		// Adding the Info label to the top component
		jpNorth.add(label, BorderLayout.NORTH);
		// Adding the indicators UI to the top component
		jpNorth.add(levelIndicators, BorderLayout.CENTER);
		// Setting up the computer goes first component
		JPanel addCompButton = new JPanel();
		// Add reference to class which would process the click
		compFirst.addActionListener(new ListenMove(-5));
		// Add label for computer goes first component
		addCompButton.add(new JLabel("Computer makes the first move ?  "));
		// Add button to the grid
		addCompButton.add(compFirst);
		// Adding the computer goes first UI to the top component
		jpNorth.add(addCompButton, BorderLayout.SOUTH);
		// Create bottom UI
		JPanel jpSouth = new JPanel();
		// Add reference to class which would process the click
		retry.addActionListener(new ListenMove(-4));
		// Add refresh button to the bottom of UI
		jpSouth.add(retry);

		// Start Laying Out the complete UI
		setLayout(new BorderLayout());
		// Layout center components
		getContentPane().add(jpCenter, BorderLayout.CENTER);
		// Layout bottom components
		getContentPane().add(jpSouth, BorderLayout.SOUTH);
		// Layout top components
		getContentPane().add(jpNorth, BorderLayout.NORTH);
		// Set size
		setSize(500, 500);
		// Set title
		setTitle("4X4 Tic-Tac-Toe by Kedar Kulkarni");
		// Display the UI
		setVisible(true);
		// Set Operation on click of exit icon
		setDefaultCloseOperation(EXIT_ON_CLOSE);
	}

	/**
	 * 
	 * This is the Listener class which receives the clicks on the UI This class
	 * manipulates the items on the UI based on what was clicked.
	 * 
	 * @author Kedar Kulkarni
	 * 
	 */
	public class ListenMove implements ActionListener {
		/** click identifer */
		int click;

		/**
		 * This is the setter method for the received click.
		 * 
		 * @param click
		 */
		public ListenMove(int click) {
			// Set this click to received value
			this.click = click;
		}

		/**
		 * This method decodes the click and manipulates the UI based on the
		 * click. It also calls the machine logic when its machines turn for
		 * play.
		 */
		public void actionPerformed(ActionEvent ae) {
			try {
				// The grid buttons causes click value of 0 to 15
				if (click >= 0) {
					// Update the button press on to machines copy
					b.updateBoard(click, nextMark);

					// Decide mark to display on the grid based on value of
					// current player
					String symbol = (nextMark == Board.Player1) ? "X" : "O";
					// Display X or O on currently clicked position.
					buttons[click].setText(symbol);
					// Change background of the clicked position
					buttons[click].setBackground(Color.LIGHT_GRAY);
					// Disable the clicked position. No more clicks are possible
					// at this position
					buttons[click].setEnabled(false);
					// Change which player makes the next move
					nextMark = (nextMark == Board.Player1) ? Board.Player2
							: Board.Player1;

					// If a manual move has been made then disable the machine
					// makes first move button if its still active
					if (compFirst.isEnabled()) {
						// Set the text displaying first move for this game was
						// not made by computer
						compFirst.setText("No");
						// Disable the computer first move button
						compFirst.setEnabled(false);
					}
					// Check if the game is over as defined in the isOver method
					// of Board class (matches the requirements details in
					// isOver Method)
					if (b.isOver()) {
						// Highlight the label color
						label.setForeground(Color.RED);
						// Set the label text to display the winner. Ask for
						// user action for new game
						label.setText("Game Over. " + b.getWinnerString()
								+ " Click Refresh for a new Game.");

						// Disable the board psoitons
						for (int i = 0; i < 16; i++) {
							// Change color of grid positions to inactive color
							buttons[i].setBackground(Color.LIGHT_GRAY);
							// Disable the positions
							buttons[i].setEnabled(false);
						}
						// Disable the level selector buttons
						for (int i = 0; i < 3; i++) {
							levels[i].setEnabled(false);
						}
						// Return focus to the board and wait for user action
						return;
					}
					// Indicate that the next move is made by machine or not.
					// This basically calls the logic for making machine move
					// after every human move
					machinePlays = !machinePlays;

					// If the next move is made by machine
					if (machinePlays) {
						// Call function which makes the machines move
						machinesMove();
					} else { // else its humans turn
						// Ask user to make a move. Return focus to the board
						// and wait for user action
						label.setText("Please make a move.");
					}

				} else { // One of the boards configuration setup buttons
					// were pressed
					// Switch based on what was pressed
					switch (click) {
					// Level Selected to Beginner
					case (-1):
						// Reset the board
						resetBoard();
						// Display text showing current difficulty level
						label.setText("Playing as a Beginner. ");
						// Change the depth of search to 2
						depth = Play.Beginner;
						// Disable the currently pressed button
						levels[0].setEnabled(false);
						// Make Default level Available
						levels[2].setEnabled(true);
						// Break out of switch-case
						break;
					// Level Selected to Intermediate
					case (-2):
						// Reset the board
						resetBoard();
						// Display text showing current difficulty level
						label.setText("Playing as an Intermediate . ");
						// Change the depth of search to 4
						depth = Play.Intermediate;
						// Disable the currently pressed button
						levels[1].setEnabled(false);
						// Make Default level Available
						levels[2].setEnabled(true);
						// Break out of switch-case
						break;
					// Level Selected to Intermediate
					case (-3):
						// Reset the board
						resetBoard();
						// Display text showing current difficulty level
						label.setText("Playing as an Expert. ");
						// Change the depth of search to 6
						depth = Play.Expert;
						// Disable the currently pressed button
						levels[2].setEnabled(false);
						// Break out of switch-case
						break;
					// Reset was pressed
					case (-4):
						// Reset the board
						resetBoard();
						// Break out of switch-case
						break;
					// Computer was asked to make the first move
					case (-5):
						// Disable the currently pressed button
						compFirst.setEnabled(false);
						// Toggle the machinePlays flag. Indicate that this move
						// is made by machine so next move would be human.
						machinePlays = true;
						// Call function which makes the machines move
						machinesMove();
						// Break out of switch-case
						break;
					}
				}
				// Catch any run time Exception
			} catch (Exception e) {
				// Log Exception to console
				e.printStackTrace();
			}
		}

		/**
		 * This method resets the board.
		 */
		public void resetBoard() {

			// Reset Label Color
			label.setForeground(Color.BLACK);
			// Show default label message
			label.setText(newGameMssg);

			// Reset all the levels the levels
			for (int i = 0; i < 3; i++) {
				// Enable all levels
				levels[i].setEnabled(true);
			}
			// Set default level to Expert
			levels[2].setEnabled(false);
			// Change the depth of search to 6 corresponding to expert level
			depth = Play.Expert;
			// Reset the machines copy of the board
			b = new Board();
			// Reset the position buttons on the UI
			for (int i = 0; i < 16; i++) {
				// Remove all X and O on the grid
				buttons[i].setText("");
				// Change color
				buttons[i].setBackground(Color.WHITE);
				// Enable the positions
				buttons[i].setEnabled(true);
			}
			// Reset the text for Computer First Move
			compFirst.setText("Yes");
			// Enable the Computer First Move
			compFirst.setEnabled(true);
			// Reset the default first move maker to human
			machinePlays = false;
			// Change that next move maker is player 1 this decide what mark to
			// show
			nextMark = Board.Player1;
		}
	}

	/**
	 * This method invokes logic for deciding the machines move
	 */
	public void machinesMove() {
		// Create object of the machines thinking class
		DecideMark newObj = new DecideMark();
		// Pass the current board positions and the next player. This returns
		// which position has to be marked on the grid
		int move = newObj.returnMove(b, nextMark);
		// Make the move by clicking returned move position on the grid
		buttons[move].doClick();
	}

	/**
	 * The Entry point in the program
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		// Create a object of the Play class.
		// This calls the UI constructor and starts the game
		new Play();

	}
}
