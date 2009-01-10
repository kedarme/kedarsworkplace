import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class usrbincrash2 {
	static Map<Integer, Integer> dp = new LinkedHashMap<Integer, Integer>();
	static List<Item> itemList;
	/** The weight to loose */
	static Integer weightToLoose = null;

	public static void main(String[] args) throws Exception {
		dp.put(0, 0);
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
		itemList = ds.getItemList();
		// Sort by weight/cost
		Collections.sort(itemList, new ItemComparator());
		// Get length of inventory
		Integer noOfItems = itemList.size();
		System.out.println("No of items=" + noOfItems);

		int res = -1, cost = -1;
		while (res < weightToLoose) {
			System.out.println("here:[res" + res + "][weightToLoose="
					+ weightToLoose + "]");
			cost = cost + 1;
			res = getMaxWeight(cost);
		}
		System.out.println("Final Cost:" + cost);
		//System.out.println(dp.toString());
	}

	private static int getMaxWeight(int cost) {
		if (cost < 0) {
			return -1;
		}
		if (dp.containsKey(cost)) {
			return dp.get(cost);
		}

		System.out.println("Size: " + dp.size());

		int best = -1;
		int currentWeight = 0;
		int currentCost = 0;
		// int best = -1;
		for (int ii = 0; ii < itemList.size(); ii++) {
			int price = itemList.get(ii).getPrice();
			int cur = getMaxWeight(cost - price);
			System.out.println("pre cur" + currentCost + " cost:" + cost
					+ " price=" + price);
			if (cur < 0)
				continue;

			cur = cur + itemList.get(ii).getWeight();

			// #define zmax(a,b) (((a)>(b))?(a):(b))
			// TODO:Confirm best = zmax(cur, best);
			 best = (cur > best ) ? cur : best ;
		}
		dp.put(cost, best);
		
		System.out.println("Returning:" + best);
		return best;

	}
}
