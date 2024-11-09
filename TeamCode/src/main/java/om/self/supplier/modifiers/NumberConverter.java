package om.self.supplier.modifiers;

import om.self.supplier.core.Modifier;
import om.self.supplier.core.Utils;

public class NumberConverter<T extends Number, R> implements Modifier<T, R> {
    private final Class<R> ref;

    public NumberConverter(Class<R> outputRef){
        this.ref = outputRef;
    }

    public NumberConverter(Class<T> inputRef, Class<R> outputRef){
        this.ref = outputRef;
    }

    @Override
    public R apply(Number t) {
        return Utils.convertNumber(t, ref);
    }
}
