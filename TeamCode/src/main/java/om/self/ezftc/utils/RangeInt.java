package om.self.ezftc.utils;

public class RangeInt extends Range<Integer>{

    public RangeInt(Integer min, Integer max) {
        super(min, max);
    }

    /**
     * takes the val and limits it to be within the min and max values of the range
     *
     * @param val the value to limit
     * @return the constrained value
     */
    @Override
    public Integer limit(Integer val) {
        return Math.min(Math.max(val, min), max);
    }

    /**
     * converts a value to unit value
     * @param val a value in range min - max
     * @return a value in range 0 - 1
     */
    @Override
    public Integer convertTo(Integer val){
        return (val - min)/(max - min);
    }

    /**
     * converts a unit value to value
     * @param val a value in range 0 - 1
     * @return a value in range min - max
     */
    @Override
    public Integer convertFrom(Integer val){
        return val * (max - min) + min;
    }

    /**
     * checks if the passed in value is within the range stored
     *
     * @param val the value to check
     * @return whether the value is in range
     */
    @Override
    public boolean isInLimit(Integer val) {
        return min < val && val < max;
    }
}
