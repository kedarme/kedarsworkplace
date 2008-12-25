package app;

import java.util.Comparator;

public class RecordsComparator implements Comparator<Record> {

	@Override
	public int compare(Record obj1, Record obj2) {
		return obj1.getSumPrice().compareTo(obj2.getSumPrice());
	}

}
