package om.self.supplier.modifiers;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.function.Function;

import om.self.supplier.core.SingleTypeModifier;

public class ModifierStack<T> implements SingleTypeModifier<T> {
    private LinkedList<Function<T, T>> stack = new LinkedList<>();

    public ModifierStack() {
    }

    public ModifierStack(LinkedList<Function<T, T>> stack) {
        this.stack = stack;
    }

    @SafeVarargs
    public ModifierStack(Function<T, T>... functions) {
        addToStack(functions);
    }

    public LinkedList<Function<T, T>> getStack() {
        return stack;
    }

    public void setStack(LinkedList<Function<T, T>> stack) {
        this.stack = stack;
    }

    public void addToStack(Function<T, T> function) {
        stack.add(function);
    }

    public void addToStack(Function<T, T>... functions) {
        stack.addAll(Arrays.asList(functions));
    }

    public void removeFromStack(int location){
        stack.remove(location);
    }

    public void removeFromStack(Function<T, T> function){
        stack.remove(function);
    }

    @Override
    public T apply(T baseInput) {
        for (Function<T, T> func : stack) {
            baseInput = func.apply(baseInput);
        }
        return baseInput;
    }
}
