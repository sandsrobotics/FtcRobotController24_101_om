package om.self.supplier.modifiers;

import java.util.function.Function;
import java.util.function.Supplier;

import om.self.supplier.core.SingleTypeModifier;

/**
 * use this instead of {@link ConditionalModifier} if the return and input types are the same because it runs faster.
 * @param <T> the type of the input and output
 */
public class SingleTypeConditionalModifier<T> implements SingleTypeModifier<T> {
    Supplier<Boolean> condition;
    Function<T, T> modification;

    public SingleTypeConditionalModifier() {
    }

    public SingleTypeConditionalModifier(Supplier<Boolean> condition, Function<T, T> modification) {
        this.condition = condition;
        this.modification = modification;
    }

    public Supplier<Boolean> getCondition() {
        return condition;
    }

    public void setCondition(Supplier<Boolean> condition) {
        this.condition = condition;
    }

    public Function<T, T> getModification() {
        return modification;
    }

    public void setModification(Function<T, T> modification) {
        this.modification = modification;
    }

    @Override
    public T apply(T baseInput) {
        return condition.get() ? modification.apply(baseInput) : baseInput;
    }
}
