package app;

import java.io.File;
import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

/**
 * This class reads the data file based on information passed in list of
 * {@link base.app.InputFile} classes. This data is used to instantiate and
 * create a list of Person objects. This is the master data list. The Columns
 * list is used with Class reflection to invoke the setter methods of
 * {@link base.app.Person} Object.
 * 
 * @author Kedar Kulkarni
 * 
 */
public class DataFileScanner {
	/** The Item data master list */
	List<Item> itemList = null;
	Integer weightToLoose = null;

	public DataFileScanner() throws ClassNotFoundException {
		// Initialize master data list.
		itemList = new ArrayList<Item>();
	}

	public void scanFile(String fileName) throws FileNotFoundException,
			NoSuchMethodException, InvocationTargetException,
			IllegalAccessException {
		// Initialize Scanner class for the given file.
		Scanner scanner = new Scanner(new File(fileName));
		// Iterate through the file line by line.
		while (scanner.hasNextLine()) {
			// Read one line as a String.
			String line = scanner.nextLine();
			// Initialize Scanner object for the record and uses default delimiter.
			Scanner lineScanner = new Scanner(line);
			if (lineScanner.hasNextInt()) { // We are reading the first line
				weightToLoose = lineScanner.nextInt();
			} else {
				Item newItem = new Item();
				if (lineScanner.hasNext()) { // SKU
					newItem.setSku(lineScanner.next().trim());
				}
				if (lineScanner.hasNext()) { // SKU
					newItem.setWeight(lineScanner.nextInt());
				}
				if (lineScanner.hasNext()) { // SKU
					newItem.setPrice(lineScanner.nextInt());
				}
				itemList.add(newItem);
			}
			lineScanner.close();
		}
		scanner.close();
	}
	public Integer getWeightToLoose() {
		return this.weightToLoose;
	}
	public List<Item> getItemList() {
		return this.itemList;
	}
}