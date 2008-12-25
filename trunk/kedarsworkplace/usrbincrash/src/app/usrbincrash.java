package app;

import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;

public class usrbincrash {
	static Integer weightToLoose;

	/**
	 * @param args
	 */
	public static void main(String[] args) throws ClassNotFoundException,
			FileNotFoundException, NoSuchMethodException,
			InvocationTargetException, IllegalAccessException {
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
		printList(itemList);
		System.out.println("++++++++");
		Collections.sort(itemList, new ItemComparator());
		printList(itemList);
		// Get length of inventory
		Integer noOfItems = itemList.size();
		// List of records
		List<Record> records = new ArrayList<Record>();
		// Create a dummy record for first sweep. This will create the records
		// for first item.
		records.add(new Record(0, 0, new Hashtable<String, Integer>()));
		// Temporary list of records for modification
		List<Record> recordsTemp = new ArrayList<Record>();

		// Loop through existing item list
		for (int i = 0; i < noOfItems; i++) {
			recordsTemp.clear();
			Item tempItem = itemList.get(i);
			//Minimize the records
			minimiser(records);
			// Get all possible cases already considered.
			Integer recordsSize = records.size();
			// Loop through the older cases
			for (int k = 0; k < recordsSize; k++) {
				Record oldRecord = (Record) records.get(k);
				Hashtable<String, Integer> oldCases = oldRecord.getCases();
				Integer oldPrice = oldRecord.getSumPrice();
				Integer oldWeight = oldRecord.getSumWeight();
				Integer oldWeightConstant = oldWeight;
				Integer x = 0;
				// Max number of current items to make the weight
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
					Record tempRec = new Record(sumPrice, sumWeight, newCases);
					// Add record to list of records
					recordsTemp.add(tempRec);
				}

			}
			// Delete all old records
			records.clear();
			// Update with new records
			records.addAll(recordsTemp);
		}
		Integer initS = records.size();
		System.out.println("Initial size=" + initS);
		// Prune records which have weight less than required
		Iterator<Record> it = records.iterator();
		while (it.hasNext()) {
			Record tempRecord = (Record) it.next();
			if (tempRecord.getSumWeight() < weightToLoose) {
				it.remove();
			}

		}
		Integer initF = records.size();
		System.out.println("Stripped size=" + initF);
		Collections.sort(records, new RecordsComparator());
		printListRecord(records);
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
		// Sort the current record stack by Price
		Collections.sort(list, new RecordsComparator());
		// Get iterator over current records
		Iterator<Record> it = list.iterator();
		// Set minPrice to lowest possible
		Integer minPrice = 0;
		while (it.hasNext()) {
			// Get new Record
			Record newRecord = (Record) it.next();
			// Note weight
			Integer recordWeight = newRecord.getSumWeight();
			// Check if this record can be operated on
			if (recordWeight >= weightToLoose) {
				Integer recordPrice = newRecord.getSumPrice();
				// Price has'nt been updated. This is first qualifier record.
				// Set Price as min Price
				if (minPrice == 0) {
					minPrice = recordPrice;
				} else if (recordPrice < minPrice) {
					// We have a record with price lower than our
					// minPrice
					minPrice = recordPrice;
				} else {
					// We have a qualifier weight with price more than current
					// minimum price. Remove it
					it.remove();
				}
			}

		}
	}

	private static void printList(List<Item> list) {
		Iterator<Item> it = list.iterator();
		while (it.hasNext()) {
			Item newItem = (Item) it.next();
			System.out.println("[SKU=" + newItem.getSku() + "][Weight="
					+ newItem.getWeight() + "][Price=" + newItem.getPrice()
					+ "]");
		}
	}

	private static void printListRecord(List<Record> list) {
		Iterator<Record> it = list.iterator();
		while (it.hasNext()) {
			Record newRecord = (Record) it.next();
			printRecord(newRecord, "*************************");
		}
	}

	private static void printRecord(Record newRecord, String seperator) {
		Hashtable<String, Integer> cases = newRecord.getCases();
		System.out.println(seperator);
		System.out.println("[Price=" + newRecord.getSumPrice() + "] [Weight="
				+ newRecord.getSumWeight() + "]");
		System.out.println("{Case Values:");
		Enumeration<String> e = cases.keys();
		while (e.hasMoreElements()) {
			String key = e.nextElement();
			System.out.println("[Key=" + key + "] [Value=" + cases.get(key)
					+ "]}");
		}
	}

}
