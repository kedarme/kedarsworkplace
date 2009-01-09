import java.io.File;
import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

/**
 * This class reads the data file invoked in the scanFile method. The dumpWeight
 * and inventory list is set which can be read by corresponding geter methods
 * 
 * @author Kedar Kulkarni
 * 
 */
public class DataFileScanner {
	/** The Item data master list */
	List<Item> itemList = null;
	/** Dump Weight */
	Integer weightToLoose = null;

	/**
	 * Class constructor initializes itemList
	 * 
	 * @throws ClassNotFoundException
	 */
	public DataFileScanner() throws ClassNotFoundException {
		// Initialize master data list.
		itemList = new ArrayList<Item>();
	}

	/**
	 * This method reads the file specified in the argument
	 * 
	 * @param fileName
	 * @throws FileNotFoundException
	 * @throws NoSuchMethodException
	 * @throws InvocationTargetException
	 * @throws IllegalAccessException
	 */
	public void scanFile(String fileName) throws FileNotFoundException,
			NoSuchMethodException, InvocationTargetException,
			IllegalAccessException {
		// Initialize Scanner class for the given file.
		Scanner scanner = new Scanner(new File(fileName));
		// Iterate through the file line by line.
		while (scanner.hasNextLine()) {
			// Read one line as a String.
			String line = scanner.nextLine();
			// Initialize Scanner object for the record and uses default
			// delimiter.
			Scanner lineScanner = new Scanner(line);
			if (lineScanner.hasNextInt()) { // We are reading the first line
				// Set the dump weight
				weightToLoose = lineScanner.nextInt();
			} else { // We are reading the inventory list
				Item newItem = new Item();
				if (lineScanner.hasNext()) { // Read SKU
					newItem.setSku(lineScanner.next().trim());
				}
				if (lineScanner.hasNext()) { // Read Weight
					newItem.setWeight(lineScanner.nextInt());
				}
				if (lineScanner.hasNext()) { // Read Price
					newItem.setPrice(lineScanner.nextInt());
				}
				// Add item to inventory list
				itemList.add(newItem);
			}
			lineScanner.close();
		}
		scanner.close();
	}

	/**
	 * Method returns dump weight
	 * 
	 * @return
	 */
	public Integer getWeightToLoose() {
		return this.weightToLoose;
	}

	/**
	 * Method returns dump weight
	 * 
	 * @return
	 */
	public List<Item> getItemList() {
		return this.itemList;
	}
}