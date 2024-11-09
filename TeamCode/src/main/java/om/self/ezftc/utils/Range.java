package om.self.ezftc.utils;

public abstract class Range<T>{
	T min;
	T max;

	public Range(T min, T max){
		this.min = min;
		this.max = max;
	}

	/**
	 * takes the val and limits it to be within the min and max values of the range
	 * @param val the value to limit
	 * @return the constrained value
	 */
	public abstract T limit(T val);

	/**
	 * converts a value to unit value
	 * @param val a value in range min - max
	 * @return a value in range 0 - 1
	 */
	public abstract T convertTo(T val);

	/**
	 * converts a unit value to value
	 * @param val a value in range 0 - 1
	 * @return a value in range min - max
	 */
	public abstract T convertFrom(T val);

	/**
	 * converts a value from the second Range range to an equivalent value in the first Range range
	 * @param val the value to convert in range min - max of second Range
	 * @param range2 the second Range for converting
	 * @return the value converted to the first end point space in range min - max
	 */
	public T doubleConvert(T val, Range<T> range2){
		return convertFrom(range2.convertTo(val));
	}

	/**
	 * checks if the passed in value is within the range stored
	 * @param val the value to check
	 * @return whether the value is in range
	 */
	public abstract boolean isInLimit(T val);
}
