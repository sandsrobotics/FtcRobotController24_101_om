package om.self.supplier.modifiers;

import java.util.function.Function;

import om.self.supplier.core.SingleTypeModifier;


public class DeadZoneModifier<T extends Comparable<T>> implements SingleTypeModifier<T> {
    private Function<T, T> deadZoneFunction;
    private T deadZoneMin;
    private T deadZoneMax;
    private boolean inDeadZone = false;

    public DeadZoneModifier() {
    }

    public DeadZoneModifier(Function<T, T> deadZoneFunction) {
        this.deadZoneFunction = deadZoneFunction;
    }

    public DeadZoneModifier(T deadZoneMin, T deadZoneMax) {
        this.deadZoneMin = deadZoneMin;
        this.deadZoneMax = deadZoneMax;
    }

    public DeadZoneModifier(Function<T, T> deadZoneFunction, T deadZoneMin, T deadZoneMax) {
        this.deadZoneFunction = deadZoneFunction;
        this.deadZoneMin = deadZoneMin;
        this.deadZoneMax = deadZoneMax;
    }

    public Function<T, T> getDeadZoneFunction() {
        return deadZoneFunction;
    }

    public void setDeadZoneFunction(Function<T, T> deadZoneFunction) {
        this.deadZoneFunction = deadZoneFunction;
    }

    public T getDeadZoneMin() {
        return deadZoneMin;
    }

    public void setDeadZoneMin(T deadZoneMin) {
        this.deadZoneMin = deadZoneMin;
    }

    public T getDeadZoneMax() {
        return deadZoneMax;
    }

    public void setDeadZoneMax(T deadZoneMax) {
        this.deadZoneMax = deadZoneMax;
    }

    public void setDeadZones(T deadZoneMin, T deadZoneMax){
        this.deadZoneMin=deadZoneMin;
        this.deadZoneMax=deadZoneMax;
    }

    public boolean isInDeadZone() {
        return inDeadZone;
    }

    @Override
    public T apply(T baseInput) {
        inDeadZone = baseInput.compareTo(deadZoneMin) < 0 || baseInput.compareTo(deadZoneMax) > 0;

        if(inDeadZone) return deadZoneFunction.apply(baseInput);
        return baseInput;
    }
}
