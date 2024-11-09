package om.self.supplier.modifiers;

import java.util.function.Function;
import java.util.function.Supplier;

import om.self.supplier.core.Modifier;

public class ConditionalModifier<T, R> implements Modifier<T, R> {
    private Supplier<Boolean> condition;
    private Function<T, R> modification;
    private Function<T, R> base;

    public ConditionalModifier() {
    }

    public ConditionalModifier(Supplier<Boolean> condition, Function<T,R> base, Function<T, R> modification) {
        this.condition = condition;
        this.base = base;
        this.modification = modification;
    }

    public Supplier<Boolean> getCondition() {
        return condition;
    }

    public void setCondition(Supplier<Boolean> condition) {
        this.condition = condition;
    }

    public Function<T, R> getModification() {
        return modification;
    }

    public void setModification(Function<T, R> modification) {
        this.modification = modification;
    }

    public Function<T, R> getBase() {
        return base;
    }

    public void setBase(Function<T, R> base) {
        this.base = base;
    }

    @Override
    public R apply(T baseInput) {
        return condition.get() ? modification.apply(baseInput) : base.apply(baseInput);
    }
}
