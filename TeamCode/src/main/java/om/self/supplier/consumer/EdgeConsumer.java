package om.self.supplier.consumer;

import java.util.function.Consumer;

public class EdgeConsumer implements Consumer<Boolean> {
    private boolean currVal;
    private boolean lastVal;

    private Runnable onRise = () -> {};

    private Runnable onFall = () -> {};

    public EdgeConsumer(){}

    public EdgeConsumer(Runnable onRise, Runnable onFall) {
        this.onRise = onRise;
        this.onFall = onFall;
    }

    public Runnable getOnRise() {
        return onRise;
    }

    public void setOnRise(Runnable onRise) {
        if (onRise == null) throw new IllegalArgumentException("onRise can not be null");
        this.onRise = onRise;
    }

    public Runnable getOnFall() {
        return onFall;
    }

    public void setOnFall(Runnable onFall) {
        if (onFall == null) throw new IllegalArgumentException("onFall can not be null");
        this.onFall = onFall;
    }

    @Override
    public void accept(Boolean value) {
        lastVal = currVal;
        currVal = value;

        if(isRisingEdge()) onRise.run();
        else if(isFallingEdge()) onFall.run();
    }


    public boolean isRisingEdge(){
        return !lastVal && currVal;
    }

    public boolean isFallingEdge(){
        return lastVal && !currVal;
    }
}
