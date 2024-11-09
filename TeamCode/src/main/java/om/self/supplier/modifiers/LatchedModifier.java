package om.self.supplier.modifiers;

public class LatchedModifier extends EdgeModifier{
    private boolean latchValue = false;

    public LatchedModifier() {
    }

    public LatchedModifier(boolean latchValue) {
        this.latchValue = latchValue;
    }

    public boolean getLatchValue() {
        return latchValue;
    }

    public void setLatchValue(boolean latchValue) {
        this.latchValue = latchValue;
    }

    @Override
    public Boolean apply(Boolean value) {
        super.apply(value);
        if(isFallingEdge()) latchValue = !latchValue;
        return latchValue;
    }
}
