package om.self.supplier.modifiers;

import om.self.supplier.core.SingleTypeModifier;

public class EdgeModifier implements SingleTypeModifier<Boolean> {
    private boolean currVal;
    private boolean lastVal;


    public EdgeModifier(){}

    @Override
    public Boolean apply(Boolean value) {
        lastVal = currVal;
        currVal = value;
        return value;
    }

    public boolean isRisingEdge(){
        return !lastVal && currVal;
    }

    public boolean isFallingEdge(){
        return lastVal && !currVal;
    }
}
