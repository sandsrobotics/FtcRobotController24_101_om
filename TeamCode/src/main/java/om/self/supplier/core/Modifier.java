package om.self.supplier.core;

import java.util.function.Function;
import java.util.function.Supplier;

/**
 * A basic interface for all modifiers in this library. It extends the {@link Function} interface and gives you a method for turing the function into a supplier which can be useful for chaining multiple modifiers together.
 * @param <T> the input type of the function. Also, the type of the base supplier when calling toSupplier.
 * @param <R> the output type of the function. Also, the type of the returned supplier when calling toSupplier.
 * @see Modifier#toSupplier(Supplier)
 * @see ModifierImpl
 */
public interface Modifier<T, R> extends Function<T, R>{
    /**
     * Converters the function associated with the Modifier/Converter to a supplier using a base supplier.
     * @param base the supplier that supplier the values for the function
     * @return a supplier that has the values of base after applying the function
     */
    default Supplier<R> toSupplier(Supplier<T> base){
        return () -> apply(base.get());
    }
}
