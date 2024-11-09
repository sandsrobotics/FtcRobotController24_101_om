package om.self.supplier.modifiers;

public class TimeRampedModifier extends SimpleRampedModifier {
    private double rampPerMs;
    private long lastUpdateTime = System.nanoTime();

    public TimeRampedModifier(double rampPerMs) {
        this.rampPerMs = rampPerMs;
    }

    public TimeRampedModifier(double rampPerMs, double currentVal) {
        super(0, currentVal);
        this.rampPerMs = rampPerMs;
    }

    public double getRampPerMs() {
        return rampPerMs;
    }

    public void setRampPerMs(double rampPerMs) {
        this.rampPerMs = rampPerMs;
    }

    @Override
    public void setCurrentVal(double currentVal) {
        super.setCurrentVal(currentVal);
        lastUpdateTime = System.nanoTime();
    }

    @Override
    public Double apply(Double baseInput) {
        setRamp(getElapsedMs() * rampPerMs);
        return super.apply(baseInput);
    }

    public double getElapsedMs(){
        long curr = System.nanoTime();
        double val = (curr - lastUpdateTime) / 1000000.0;
        lastUpdateTime = curr;
        return val;
    }
}
