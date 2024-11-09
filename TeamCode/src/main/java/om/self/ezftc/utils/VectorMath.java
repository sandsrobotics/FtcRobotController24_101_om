package om.self.ezftc.utils;

public class VectorMath {
    public static Vector3 add(Vector3 v1, Vector3 v2){
        return new Vector3(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
    }

    public static Vector2 add(Vector2 v1, Vector2 v2){
        return new Vector2(v1.X + v2.X, v1.Y + v2.Y);
    }

    public static Vector3 subtract(Vector3 v1, Vector3 v2){
        return new Vector3(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
    }

    public static double dotProduct(Vector3 v1, Vector3 v2){
        return v1.X * v2.X + v1.Y * v2.Y + v1.Z + v2.Z;
    }

    public static Vector3 multiply(Vector3 v1, Vector3 v2){
        return new Vector3(v1.X * v2.X, v1.Y * v2.Y, v1.Z * v2.Z);
    }

    public static Vector3 multiply(Vector3 v1, double val){
        return new Vector3(v1.X * val, v1.Y * val, v1.Z * val);
    }

    public static Vector3 multiplyAsVector2(Vector3 v1, double val){
        return new Vector3(v1.X * val, v1.Y * val, v1.Z);
    }

    public static Vector3 divide(Vector3 v1, double val){
        return new Vector3(v1.X / val, v1.Y / val, v1.Z / val);
    }

    /**
     * translates a vector 3 along the x and y axis by treating z like an angle(in degrees)
     * @return
     */
    public static Vector3 translateAsVector2(Vector3 val, double x, double y){
        double r = Math.toRadians(val.Z);
        double xOut = val.X + y * Math.cos(r) + x * Math.sin(r);
        double yOut = val.Y - x * Math.cos(r) + y * Math.sin(r);
        return new Vector3(xOut, yOut, val.Z);
    }

//    public static Vector3 translateTagAsVector2(Vector3 val, double x, double y){
//        double r = Math.toRadians(val.Z);
//        double xOut = val.X + y * Math.sin(r) + x * Math.cos(r);
//        double yOut = val.Y + x * Math.sin(r) + y * Math.cos(r);
//        return new Vector3(xOut, yOut, val.Z);
//    }

    public static Vector3 translateTagAsVector2(Vector3 val, double x, double y) {
        double r = Math.toRadians(val.Z);
        double xOut = val.X + x * Math.cos(r) - y * Math.sin(r);
        double yOut = val.Y + x * Math.sin(r) + y * Math.cos(r);
        return new Vector3(xOut, yOut, val.Z);
    }

    public static boolean inTolerance(double[] currPos, double[] targetPos, double[] tol){
        for(int i = 0; i < 3; i++)
            if(Math.abs(targetPos[i] - currPos[i]) > tol[i]) return false;
        return true;
    }

    public static boolean inTolerance(Vector3 currPos, Vector3 targetPos, Vector3 tol){
        return inTolerance(currPos.toArray(), targetPos.toArray(), tol.toArray());
    }

    public static boolean inTolerance(Vector2 currPos, Vector2 targetPos, Vector2 tol){
        return inTolerance(currPos.toArray(), targetPos.toArray(), tol.toArray());
    }
}
