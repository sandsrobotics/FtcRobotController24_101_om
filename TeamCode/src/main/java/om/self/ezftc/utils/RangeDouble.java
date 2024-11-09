package om.self.ezftc.utils;

public class RangeDouble extends Range<Double>{

    public RangeDouble(Double min, Double max) {
        super(min, max);
    }

    /**
     * creates a default range with a value of 0 to 1
     */
    public RangeDouble(){
        super(0.0,1.0);
    }

    /**
     * takes the val and limits it to be within the min and max values of the range
     *
     * @param val the value to limit
     * @return the constrained value
     */
    @Override
    public Double limit(Double val) {
        return Math.min(Math.max(val, min), max);
    }

    /**
     * converts a value to unit value
     * @param val a value in range min - max
     * @return a value in range 0 - 1
     */
    @Override
    public Double convertTo(Double val){
        return (val - min)/(max - min);
    }

    /**
     * converts a unit value to value
     * @param val a value in range 0 - 1
     * @return a value in range min - max
     */
    @Override
    public Double convertFrom(Double val){
        return val * (max - min) + min;
    }

    /**
     * checks if the passed in value is within the range stored
     *
     * @param val the value to check
     * @return whether the value is in range
     */
    @Override
    public boolean isInLimit(Double val) {
        return min < val && val < max;
    }
}
