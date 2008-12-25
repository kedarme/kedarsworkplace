package app;

import java.util.Hashtable;

public class Record {
	private Integer sumPrice;
	private Integer sumWeight;
	// Sku <=> no of Items
	Hashtable<String,Integer> cases = new Hashtable<String,Integer>();

	public Record(Integer sumPrice, Integer sumWeight,
			Hashtable<String, Integer> cases) {
		super();
		this.sumPrice = sumPrice;
		this.sumWeight = sumWeight;
		this.cases = cases;
	}
	
	public Record() {
		super();
	}
	
	public void copy(Record r) {
		this.setCases(r.getCases());
		this.setSumPrice(r.getSumPrice());
		this.setSumWeight(r.getSumWeight());
	}

	public Integer getSumPrice() {
		return sumPrice;
	}

	public void setSumPrice(Integer sumPrice) {
		this.sumPrice = sumPrice;
	}

	public Hashtable<String,Integer> getCases() {
		return cases;
	}

	public void setCases(Hashtable<String,Integer> cases) {
		this.cases = cases;
	}

	public Integer getSumWeight() {
		return sumWeight;
	}

	public void setSumWeight(Integer sumWeight) {
		this.sumWeight = sumWeight;
	}

}