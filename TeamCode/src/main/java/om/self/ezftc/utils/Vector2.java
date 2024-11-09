package om.self.ezftc.utils;

public class Vector2{
    public final double X,Y;

    public Vector2(double X, double Y){
        this.X = X;
        this.Y = Y;
    }

    public Vector2(double[] vals){
        this.X = vals[0];
        this.Y = vals[1];
    }

    public Vector2(){
        X = 0;
        Y = 0;
    }

    public double[] toArray() {
        return new double[]{X,Y};
    }

    public double get(int index){
        switch(index){
            case 1: return X;
            case 2: return Y;
            default: throw new IndexOutOfBoundsException("the index '" + index + "' is not inside a vector 2");
        }
    }

    public Vector2 switchXY(){
        return new Vector2(Y, X);
    }

    public Vector2 invert(){
        return new Vector2(-Y, -X);
    }

    public Vector2 invertX(){
        return new Vector2(-X, Y);
    }

    public Vector2 invertY(){
        return new Vector2(X, -Y);
    }

    public Vector2 withX(double X){return new Vector2(X,Y);}

    public Vector2 withY(double Y){return new Vector2(X,Y);}

    public boolean inTolerance(Vector2 targetPos, Vector2 tol){
        return VectorMath.inTolerance(this, targetPos, tol);
    }

    public boolean inTolerance(double[] targetPos, double[] tol){
        return VectorMath.inTolerance(toArray(), targetPos, tol);
    }

    @Override
    public boolean equals(Object obj) {
        if(! (obj instanceof Vector2)) return false;
        Vector2 other = (Vector2) obj;
        return other.X == X && other.Y == Y;
    }

    public String toString(int decimals){
        return "Vector2{X=" + String.format("%."+ decimals +"f", X) + ", Y=" + String.format("%."+ decimals +"f", Y) + "}";
    }

    @Override
    public String toString() {
        return toString(2);
    }
}