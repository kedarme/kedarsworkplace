package app;

import java.util.Comparator;

public class ItemComparator implements Comparator<Item> {

	@Override
	public int compare(Item obj1, Item obj2) {
		Double pricePerPound1 = (obj1.getPrice().doubleValue() / obj1.getWeight().doubleValue());
		Double pricePerPound2 = (obj2.getPrice().doubleValue() / obj2.getWeight().doubleValue());
		return pricePerPound1.compareTo(pricePerPound2);
	}

}
