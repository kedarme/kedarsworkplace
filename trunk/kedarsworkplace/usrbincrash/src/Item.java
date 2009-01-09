/**
 * This class holds the data for the an individual item in the inventory.
 * 
 * @author Kedar Kulkarni
 * 
 */
class Item {
	/** The SKU */
	private String sku;
	/** Weight of one item unit */
	private Integer weight;
	/** Price of one item unit */
	private Integer price;

	/**
	 * Method to get the SKU
	 * 
	 * @return
	 */
	public String getSku() {
		return sku;
	}

	/**
	 * Method to set the SKU
	 * 
	 * @param sku
	 */
	public void setSku(String sku) {
		this.sku = sku;
	}

	/**
	 * Method to get the weight
	 * 
	 * @return
	 */
	public Integer getWeight() {
		return weight;
	}

	/**
	 * Method to set the weight
	 * 
	 * @param weight
	 */
	public void setWeight(Integer weight) {
		this.weight = weight;
	}

	/**
	 * Method to get the Price
	 * 
	 * @return
	 */
	public Integer getPrice() {
		return price;
	}

	/**
	 * Method to set the Price
	 * 
	 * @param price
	 */
	public void setPrice(Integer price) {
		this.price = price;
	}

}