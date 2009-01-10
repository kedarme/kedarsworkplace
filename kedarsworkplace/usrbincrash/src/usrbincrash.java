import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class usrbincrash {
	/** default flag indicating impossible conditions */
	private static final int FLAG = -1;

	/** The Inventory List */
	static List<Item> itemList;

	/** Map of records indicating cost => weight mapping */
	static Map<Integer, Integer> records = new LinkedHashMap<Integer, Integer>();

	/** The weight to loose */
	static Integer weightToLoose = null;

	public static void main(String[] args) throws Exception {
		// Check if we have a filename argument passed
		if (args.length != 1) {
			System.out.println("Input expected.");
			System.exit(0);
		}
		try {
			// Begin scanning data file
			DataFileScanner ds = new DataFileScanner();
			ds.scanFile(args[0]);
			// Get how much weight needs to be dumped
			weightToLoose = ds.getWeightToLoose();
			// Get the inventory list
			itemList = ds.getItemList();
			// Create a dummy record for first sweep. This will create the
			// records for first item.
			records.put(0, 0);
			// The cost for which we try to get how much weight can be dropped
			int cost = FLAG;
			// The resultWeight for given cost
			int resultWeight = FLAG;

			// Loop till we have a weight satisfying the weight to drop
			while (resultWeight < weightToLoose) {
				// Calls function getMaxWeight with increasing costs
				resultWeight = getMaxWeight(++cost);
			}
			System.out.println("Final Cost:" + cost);
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
	 * This function returns the weight that can be dropped for a given cost. It
	 * also creates the map of cost => weight. This function calls itself
	 * iteratively to calculate the required weight value.
	 * 
	 * @param cost
	 * @return
	 */
	private static int getMaxWeight(int cost) {
		// For obvious condition of negative cost return FLAG.
		// This occurs during function iterative calls
		if (cost < 0) {
			return FLAG;
		}
		// Return the corresponding weight we calculated for this cost.
		// This step is an DP execution
		if (records.containsKey(cost)) {
			return records.get(cost);
		}

		// Set the variable for default weight. Assume it to be worst i.e.
		// invalid FLAG
		int defaultWeight = FLAG;
		// Iterate over the inventory list
		for (int index = 0; index < itemList.size(); index++) {
			// Calculate current drop weight considering current item. The
			// iterative call to this function with the price margin as cost
			// will either return invalid state ( when price of current
			// item=>total cost allocated for this iteration => passes negative
			// cost to the function) or gives a pre-calculated weight for this
			// cost if it exists in the cost=>weight map
			int currentWeight = getMaxWeight(cost
					- itemList.get(index).getPrice());
			// If we got a invalid weight state try the the next item.
			if (currentWeight < 0) {
				continue;
			}
			// Proceed here for valid weight state. Add weight of current item
			// to the current accumulated weight
			currentWeight += itemList.get(index).getWeight();
			// Check if the currentWeight > defaultWeight. This will be used to
			// fill in the cost=>weight map
			defaultWeight = (currentWeight > defaultWeight) ? currentWeight
					: defaultWeight;
		}
		// Add the current iterations cost=>weight to the map
		records.put(cost, defaultWeight);
		// Return the weight calculated for the cost passed to this iteration
		return defaultWeight;
	}
}