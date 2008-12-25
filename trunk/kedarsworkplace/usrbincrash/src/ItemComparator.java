

import java.util.Comparator;

/**
 * This class implements the Comparator for Item objects We compare based on
 * price per unit weight.
 * 
 * @author Kedar Kulkarni
 * 
 */
public class ItemComparator implements Comparator<Item> {
	/**
	 * The overidden compare method.
	 */
	@Override
	public int compare(Item obj1, Item obj2) {
		// Price per unit weight for obj1
		Double pricePerPound1 = (obj1.getPrice().doubleValue() / obj1
				.getWeight().doubleValue());
		// Price per unit weight for obj1
		Double pricePerPound2 = (obj2.getPrice().doubleValue() / obj2
				.getWeight().doubleValue());
		// Return comparison between the two
		return pricePerPound1.compareTo(pricePerPound2);
	}

}
