package app;

import java.util.Hashtable;

/**
 * This class represents each of the cases considered when calculating the best
 * combination of items for dumping. This contains the sum of weight, price of
 * all items for this record. It also has a hash table containing all item skus
 * for this record and their corresponding number.
 * 
 * @author Kedar Kulkarni
 * 
 */
public class Record {
	/** Sum of Price of all items in this record */
	private Integer sumPrice;
	/** Sum of Weight of all items in this record */
	private Integer sumWeight;
	/**
	 * Hash table containing the item SKUs in the record and corresponding
	 * number of items
	 */
	Hashtable<String, Integer> cases = new Hashtable<String, Integer>();

	/**
	 * Constructor with all fields.
	 * 
	 * @param sumPrice
	 * @param sumWeight
	 * @param cases
	 */
	public Record(Integer sumPrice, Integer sumWeight,
			Hashtable<String, Integer> cases) {
		super();
		this.sumPrice = sumPrice;
		this.sumWeight = sumWeight;
		this.cases = cases;
	}

	/**
	 * Default Empty constructor.
	 */
	public Record() {
		super();
	}

	/**
	 * Method to get the Sum of Prices
	 * 
	 * @return
	 */
	public Integer getSumPrice() {
		return sumPrice;
	}

	/**
	 * Method to set the Sum of Prices
	 * 
	 * @param sumPrice
	 */
	public void setSumPrice(Integer sumPrice) {
		this.sumPrice = sumPrice;
	}

	/**
	 * Method to get the items and their numbers in this record.
	 * 
	 * @return
	 */
	public Hashtable<String, Integer> getCases() {
		return cases;
	}

	/**
	 * Method to set the items and their numbers in this record.
	 * 
	 * @param cases
	 */
	public void setCases(Hashtable<String, Integer> cases) {
		this.cases = cases;
	}

	/**
	 * Method to get the sum of weights.
	 * 
	 * @return
	 */
	public Integer getSumWeight() {
		return sumWeight;
	}

	/**
	 * Method to set the sum of weights.
	 * 
	 * @param sumWeight
	 */
	public void setSumWeight(Integer sumWeight) {
		this.sumWeight = sumWeight;
	}

}