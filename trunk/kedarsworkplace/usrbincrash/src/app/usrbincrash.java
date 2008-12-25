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
		// Get the inventory
		List<Item> itemList = ds.getItemList();

		// System.out.println("The weightToLoose: " + weightToLoose);
		printList(itemList);
		Collections.sort(itemList, new ItemComparator());
		System.out.println("----------------------");
		printList(itemList);
		// System.out.println("----------------------");
		Integer noOfItems = itemList.size();

		List<Record> records = new ArrayList<Record>();
		// Create a dummy record for first sweep. This will create the records for first item.
		records.add(new Record(0,0,new Hashtable<String, Integer>()));
		List<Record> recordsTemp = new ArrayList<Record>();
		for (int i = 0; i < noOfItems; i++) {
			recordsTemp.clear();
			Item tempItem = itemList.get(i);
			// Remove the obvious failing cases
			recordsTemp = minimiser(records);
			records.clear();
			records.addAll(recordsTemp);
			recordsTemp.clear();

			// Get all possible cases already considered.
			Integer recordsSize = records.size();
			System.out.println("Size=" + recordsSize + "::i=" + i);
			//if (recordsSize != 0) {
				for (int k = 0; k < recordsSize; k++) {
					Record oldRecord = (Record) records.get(k);
					Hashtable<String, Integer> oldCases = oldRecord.getCases();
					Integer oldPrice = oldRecord.getSumPrice();
					Integer oldWeight = oldRecord.getSumWeight();
					Integer oldWeightConstant = oldWeight;
					Integer x = 0;
					while (oldWeight <= weightToLoose) {
						x += 1;
						oldWeight += tempItem.getWeight();
					}

					// Create List of records
					for (Integer j = 0; j <= x; j++) {
					//	System.out.println("--here--");
						Hashtable<String, Integer> newCases = new Hashtable<String, Integer>(
								oldCases);
						newCases.put(tempItem.getSku(), j);
						Integer sumPrice = oldPrice + (tempItem.getPrice() * j);
						Integer sumWeight = oldWeightConstant + (tempItem.getWeight() * j);
						Record tempRec = new Record(sumPrice, sumWeight, newCases);
						recordsTemp.add(tempRec);
						// printRecord(tempRec, "++++++++++++++++++++++++"+j);
					}

				}
			/*} else {// First Record
				Integer oldWeight = 0;
				Integer x = 0;
				while (oldWeight <= weightToLoose) {
					System.out.println("------------------------------------------------there");
					x += 1;
					oldWeight += tempItem.getWeight();
				}
				// System.out.println("X:"+x);
				for (Integer j = 0; j <= x; j++) {
					Hashtable<String, Integer> newCases = new Hashtable<String, Integer>();
					newCases.put(tempItem.getSku(), j);
					Integer sumPrice = (tempItem.getPrice() * j);
					Integer sumWeight = (tempItem.getWeight() * j);
					Record tempRec = new Record(sumPrice, sumWeight, newCases);
					recordsTemp.add(tempRec);
					// printRecord(tempRec, "++++++++++++++++++++++++"+j);
				}
			}
			*/
			// Delete all old records
			records.clear();
			// Update with new records
			records.addAll(recordsTemp);
		}
		System.out.println("Printing after loop");
		// printListRecord(records);
		//List<Record> recordsTemp = new ArrayList<Record>();
		recordsTemp.clear();
		Integer initS = records.size();
		System.out.println("Initial size=" + initS);
		Iterator<Record> it = records.iterator();
		while (it.hasNext()) {
			Record tempRecord = (Record) it.next();
			if (tempRecord.getSumWeight() >= weightToLoose) {
				recordsTemp.add(tempRecord);
			}
		}
		Integer initF = recordsTemp.size();
		System.out.println("Stripped size=" + initF);

		Collections.sort(recordsTemp, new RecordsComparator());
		printRecord(recordsTemp.get(0),"+++++++++++++++++++");
	//	printListRecord(recordsTemp);
	}

	private static List<Record> minimiser(List<Record> list) {
		List<Record> newList = new ArrayList<Record>();
		Iterator<Record> it = list.iterator();
		Record qualify = new Record(0, 0, null);
		while (it.hasNext()) {
			Record newRecord = (Record) it.next();
			Integer recordWeight = newRecord.getSumWeight();
			if (recordWeight >= weightToLoose) {
				if (qualify.getSumPrice().equals(0)
						&& qualify.getSumWeight().equals(0)) {
					qualify.copy(newRecord); // first copy
					newList.add(newRecord);
				} else if (qualify.getSumPrice() < newRecord.getSumPrice()) {
					newList.add(newRecord);
				}
			} else {
				newList.add(newRecord);
			}
		}

		return newList;
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
		System.out.println("[Price=" + newRecord.getSumPrice() + "] [Weight=" + newRecord.getSumWeight() + "]");
		System.out.println("{Case Values:");
		Enumeration<String> e = cases.keys();
		while (e.hasMoreElements()) {
			String key = e.nextElement();
			System.out.println("[Key=" + key + "] [Value=" + cases.get(key) + "]}");
		}
	}

}
