package om.self.supplier.core;

import java.util.function.Function;

/**
 * An implementation of the Modifier interface that stores a function to be applied as the modification for Modifier.
 * @param <T> the input type
 * @param <R> the return type
 * @see Modifier
 */
public class ModifierImpl<T, R> implements Modifier<T, R>{
    /**
     * the function run by this modifier
     */
    private Function<T, R> function;

    /**
     * A default no args constructor.
     * @implNote IMPORTANT: you must set the function before using this modifier else it will throw a {@link NullPointerException}.
     * @see ModifierImpl#setFunction(Function)
     */
    public ModifierImpl() {
    }

    /**
     * This is the recommended constructor that sets {@link ModifierImpl#function}.
     * @param function the function this modifier should run
     */
    public ModifierImpl(Function<T, R> function) {
        this.function = function;
    }

    /**
     * returns the set function
     * @return {@link ModifierImpl#function}
     */
    public Function<T, R> getFunction() {
        return function;
    }

    /**
     * sets the function
     * @param function the function you want this modifier to run.
     * @implNote please do not use null because it will throw a {@link NullPointerException} when function is used
     */
    public void setFunction(Function<T, R> function) {
        this.function = function;
    }

    @Override
    public R apply(T t) {
        return function.apply(t);
    }
}
