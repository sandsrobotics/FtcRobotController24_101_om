package om.self.supplier.suppliers;

import java.util.function.Function;
import java.util.function.Supplier;

public class LinkedSupplier<T, R> implements Supplier<R> {
    T input;
    Function<T, R> conversionFunction;

    public LinkedSupplier() {
    }

    public LinkedSupplier(T input) {
        this.input = input;
    }

    public LinkedSupplier(Function<T, R> conversionFunction) {
        this.conversionFunction = conversionFunction;
    }

    public LinkedSupplier(T input, Function<T, R> conversionFunction) {
        this.input = input;
        this.conversionFunction = conversionFunction;
    }

    public T getInput() {
        return input;
    }

    public void setInput(T input) {
        this.input = input;
    }

    public Function<T, R> getConversionFunction() {
        return conversionFunction;
    }

    public void setConversionFunction(Function<T, R> conversionFunction) {
        this.conversionFunction = conversionFunction;
    }

    @Override
    public R get() {
        return conversionFunction.apply(input);
    }
}
