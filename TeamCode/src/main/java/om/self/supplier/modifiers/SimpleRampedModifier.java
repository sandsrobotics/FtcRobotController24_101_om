package om.self.supplier.modifiers;

import om.self.supplier.core.SingleTypeModifier;

public class SimpleRampedModifier implements SingleTypeModifier<Double> {
    private double ramp;
    private double currentVal;

    public SimpleRampedModifier() {
    }

    public SimpleRampedModifier(double ramp) {
        this.ramp = ramp;
    }

    public SimpleRampedModifier(double ramp, double currentVal) {
        this.ramp = ramp;
        this.currentVal = currentVal;
    }

    public double getRamp() {
        return ramp;
    }

    public void setRamp(double ramp) {
        this.ramp = ramp;
    }

    public double getCurrentVal() {
        return currentVal;
    }

    public void setCurrentVal(double currentVal) {
        this.currentVal = currentVal;
    }

    @Override
    public Double apply(Double baseInput) {
        if(currentVal < baseInput) currentVal = Math.min(baseInput, currentVal + ramp);
        else if(currentVal > baseInput) currentVal = Math.max(baseInput, currentVal - ramp);

        return currentVal;
    }
}
