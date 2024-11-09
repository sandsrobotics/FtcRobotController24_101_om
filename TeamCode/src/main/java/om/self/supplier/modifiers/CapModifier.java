package om.self.supplier.modifiers;

import om.self.supplier.core.SingleTypeModifier;

/**
 * Takes the input value and caps it to be within {@link CapModifier#minCap} and {@link CapModifier#maxCap}.
 * @param <T> the type of the input and output
 * @see SingleTypeModifier
 */
public class CapModifier <T extends Comparable<T>> implements SingleTypeModifier<T> {
    /**
     * the minimum value(inclusive) that can be returned
     */
    private T minCap;

    /**
     * the maximum value(inclusive) that can be returned
     */
    private T maxCap;

    private boolean belowRange = false;
    private boolean aboveRange = false;

    /**
     * A default no args constructor.
     * @implNote IMPORTANT: you must set the min and max cap before using this modifier else it will throw a {@link NullPointerException}.
     * @see CapModifier#setCaps(Comparable, Comparable)
     */
    public CapModifier() {}

    /**
     * This is the recommended constructor that sets {@link CapModifier#minCap} and {@link CapModifier#maxCap}.
     * @param minCap the minimum value(inclusive) that it can be
     * @param maxCap the maximum value(inclusive) that it can be
     */
    public CapModifier(T minCap, T maxCap) {
        this.minCap = minCap;
        this.maxCap = maxCap;
    }

    /**
     * returns the current minCap
     * @return {@link CapModifier#minCap}
     */
    public T getMinCap() {
        return minCap;
    }

    public void setMinCap(T minCap) {
        this.minCap = minCap;
    }

    /**
     * returns the current maxCap
     * @return {@link CapModifier#maxCap}
     */
    public T getMaxCap() {
        return maxCap;
    }

    public void setMaxCap(T maxCap) {
        this.maxCap = maxCap;
    }

    /**
     * sets both the min(inclusive) and max(inclusive) values
     * @param minCap the minimum the return can be
     * @param maxCap the maximum the return can be
     */
    public void setCaps(T minCap, T maxCap){
        this.minCap = minCap;
        this.maxCap = maxCap;
    }

    /**
     * if the last value inputted was within the specified range
     * @return if the last value was in range(inclusive)
     */
    public boolean isInRange() {
        return !(belowRange || aboveRange);
    }

    /**
     * if the last value inputted was below the specified range
     * @return if the last value was below range(exclusive)
     */
    public boolean isBelowRange() {
        return belowRange;
    }

    /**
     * if the last value inputted was above the specified range
     * @return if the last value was above range(exclusive)
     */
    public boolean isAboveRange(){
        return aboveRange;
    }

    @Override
    public T apply(T baseInput) {
        belowRange = baseInput.compareTo(minCap) < 0;
        if(belowRange){
            aboveRange = false;
            return minCap;
        }
        aboveRange = baseInput.compareTo(maxCap) > 0;
        if(aboveRange) return maxCap;
        return baseInput;
    }
}
