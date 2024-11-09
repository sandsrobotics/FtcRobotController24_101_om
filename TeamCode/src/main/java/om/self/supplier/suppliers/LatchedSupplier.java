package om.self.supplier.suppliers;

import java.util.function.Supplier;

public class LatchedSupplier extends EdgeSupplier {
    private boolean latchValue = false;

    public LatchedSupplier() {
    }

    public LatchedSupplier(boolean latchValue) {
        this.latchValue = latchValue;
    }

    public LatchedSupplier(Supplier<Boolean> baseSupplier, boolean latchValue){
        this.latchValue = latchValue;
        this.setBase(baseSupplier);
    }

    public boolean getLatchValue() {
        return latchValue;
    }

    public void setLatchValue(boolean latchValue) {
        this.latchValue = latchValue;
    }

    public Boolean get() {
        if(isFallingEdge()) latchValue = !latchValue;
        return latchValue;
    }
}
