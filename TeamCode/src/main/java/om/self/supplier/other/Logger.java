package om.self.supplier.other;

import java.util.LinkedList;
import java.util.function.Function;

public class Logger <T> implements Function<T, T> {
    private final LinkedList<T> log = new LinkedList<>();
    private boolean loggingEnabled = true;

    public LinkedList<T> getLog() {
        return log;
    }

    public boolean isLoggingEnabled() {
        return loggingEnabled;
    }

    public void setLoggingEnabled(boolean loggingEnabled) {
        this.loggingEnabled = loggingEnabled;
    }

    @Override
    public T apply(T t) {
        if(loggingEnabled)
            log.add(t);
        return t;
    }
}
