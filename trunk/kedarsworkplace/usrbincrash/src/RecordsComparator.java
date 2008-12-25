

import java.util.Comparator;

/**
 * This class implements the Comparator for Record objects. We compare based on
 * price.
 * 
 * @author Kedar Kulkarni
 * 
 */
public class RecordsComparator implements Comparator<Record> {
	/**
	 * The overidden compare method.
	 */
	@Override
	public int compare(Record obj1, Record obj2) {
		// Return comparison between the prices.
		return obj1.getSumPrice().compareTo(obj2.getSumPrice());
	}

}
