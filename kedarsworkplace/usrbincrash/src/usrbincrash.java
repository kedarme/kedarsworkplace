import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;

/**
 * This is the main entry point for the "usrbincrash" problem.
 * 
 * @author Kedar Kulkarni
 * 
 */
public class usrbincrash {
	/** The weight to loose */
	static Integer weightToLoose = null;

	/**
	 * The main method. Takes the file name with first line weight to dump and
	 * the rest of the lines is list of inventory.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		try {
			long sTime = new Date().getTime();
			// Check if we have a filename argument passed
			if (args.length != 1) {
				System.out.println("Input expected.");
				System.exit(0);
			}
			// Begin scanning data file
			DataFileScanner ds = new DataFileScanner();
			ds.scanFile(args[0]);
			// Get how much weight needs to be dumped
			weightToLoose = ds.getWeightToLoose();
			// Get the inventory list
			List<Item> itemList = ds.getItemList();
			// Sort by weight/cost
			Collections.sort(itemList, new ItemComparator());
			// Get length of inventory
			Integer noOfItems = itemList.size();
			// List of records
			List<Record> records = new ArrayList<Record>();
			// Create a dummy record for first sweep. This will create the
			// records
			// for first item.
			records.add(new Record(0, 0, new Hashtable<String, Integer>()));
			// Temporary list of records for modification
			List<Record> recordsTemp = new ArrayList<Record>();

			// Loop through existing item list
			for (int i = 0; i < noOfItems; i++) {
				recordsTemp.clear();
				Item tempItem = itemList.get(i);
				// Minimize the records
				minimiser(records);
				// Get all possible cases already considered.
				Integer recordsSize = records.size();
				// Loop through the older cases
				for (int k = 0; k < recordsSize; k++) {
					// Get current record
					Record oldRecord = (Record) records.get(k);
					// Get current cases
					Hashtable<String, Integer> oldCases = oldRecord.getCases();
					// Get Price of the current record
					Integer oldPrice = oldRecord.getSumPrice();
					// Get Weight of the current record
					Integer oldWeight = oldRecord.getSumWeight();
					// Set a constant for the weight of current record
					Integer oldWeightConstant = oldWeight;
					// Initiate a constant for number of current items to make
					// the
					// dump weight
					Integer x = 0;
					// Max number of current items to make the dump weight
					while (oldWeight <= weightToLoose) {
						x += 1;
						oldWeight += tempItem.getWeight();
					}

					// Create New Possible combinations leading to max number of
					// current items
					for (Integer j = 0; j <= x; j++) {
						// Start creating new case with existing older cases.
						Hashtable<String, Integer> newCases = new Hashtable<String, Integer>(
								oldCases);
						// Append current item to list of cases
						newCases.put(tempItem.getSku(), j);
						// Update the Price of the record
						Integer sumPrice = oldPrice + (tempItem.getPrice() * j);
						// Update the Weight of the record
						Integer sumWeight = oldWeightConstant
								+ (tempItem.getWeight() * j);
						// Create new record item
						Record tempRec = new Record(sumPrice, sumWeight,
								newCases);
						// Add record to list of records
						recordsTemp.add(tempRec);
					}

				}
				// Delete all old records
				records.clear();
				// Update with new records
				records.addAll(recordsTemp);
			}
			// Prune records which have weight less than dump weight
			Iterator<Record> it = records.iterator();
			while (it.hasNext()) {
				Record tempRecord = (Record) it.next();
				if (tempRecord.getSumWeight() < weightToLoose) {
					it.remove();
				}

			}
			Collections.sort(records, new RecordsComparator());
			System.out.println(records.get(0).getSumPrice());
			System.out.println("[Time="+(new Date().getTime() - sTime)+"ms][No of Records Generated="+records.size()+"]");
		}/* Catch all possible Exception in the program */
		catch (ClassNotFoundException cnfe) {
			System.out.println("Exception: " + cnfe.toString());
		} catch (FileNotFoundException iae) {
			System.out.println("Exception: " + iae.toString());
		} catch (NoSuchMethodException nsme) {
			System.out.println("Exception: " + nsme.toString());
		} catch (IllegalAccessException iae) {
			System.out.println("Exception: " + iae.toString());
		} catch (InvocationTargetException ite) {
			System.out.println("Exception: " + ite.toString());
		} catch (Exception e) {
			// Catch any Exception we didn't plan for.
			System.out.println("Exception: " + e.toString());
		}
	}

	/**
	 * This method minimizes the available records at every stage. It sorts
	 * current record by price and then prunes away records which are meet dump
	 * requirement but are costlier then our minimum priced record.
	 * 
	 * This is based on the fact that any record which meets dump weight already
	 * will not be modified by addition of next items.
	 * 
	 * @param list
	 */
	 private static void minimiser(List<Record> list) {
		 Iterator<Record> it = list.iterator();
		 Integer minPrice = 0;
		 // 1st Sweep Sets minPrice
		 while (it.hasNext()) {
			 Record newRecord = (Record) it.next();
			 Integer recordWeight = newRecord.getSumWeight();
			 if (recordWeight >= weightToLoose) {
				 Integer recordPrice = newRecord.getSumPrice();
				 if ( ( recordPrice < minPrice ) || ( minPrice == 0 ) ) {
					 minPrice = recordPrice; 
				 }
			 }
		 }
		// 2nd Sweep remove records
		it = list.iterator();
		 while (it.hasNext()) {
			 Record newRecord = (Record) it.next();
				 Integer recordPrice = newRecord.getSumPrice();
				 if ( recordPrice > minPrice ) {
					 it.remove(); 
				 }
		 }
	 }
}
