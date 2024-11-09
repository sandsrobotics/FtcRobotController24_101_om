package om.self.ezftc.utils;

public interface Vector<T> {
    int getSize();

    T add(T v2);
    T subtract(T v2);
    T multiply(double scale);
    T divide(double scale);
    T matrixMultiply(T v2);

    T getDistances(T v2);
    double getDistance(T v2);
    T getAngle(T v2);

    double dotProduct(T v2);
    T crossProduct(T v2);

    double[] asArray();
}
