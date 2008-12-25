package app;

class Item { 
	
	private String sku;
	private Integer weight;
	private Integer price;
	public Item() {
		super();
	}
	public Item(String sku, Integer weight, Integer price) {
		super();
		this.sku = sku;
		this.weight = weight;
		this.price = price;
	}
	public String getSku() {
		return sku;
	}
	public void setSku(String sku) {
		this.sku = sku;
	}
	public Integer getWeight() {
		return weight;
	}
	public void setWeight(Integer weight) {
		this.weight = weight;
	}
	public Integer getPrice() {
		return price;
	}
	public void setPrice(Integer price) {
		this.price = price;
	}
	
}