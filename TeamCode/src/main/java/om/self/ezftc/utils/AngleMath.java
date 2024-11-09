package om.self.ezftc.utils;

public class AngleMath {
    public static double findAngleError(double currentAngle, double targetAngle)
    {
        targetAngle = scaleAngle(targetAngle);
        double angleError = currentAngle - targetAngle;
        if (angleError > 180) {
            angleError -= 360;
        } else if (angleError < -180) {
            angleError += 360;
        }
        return angleError;  //LK -
    }

    public static double scaleAngle(double angle)// scales an angle to fit in -180 to 180
    {
        angle = angle % 360;
        if (angle > 180) { return angle - 360; }
        if (angle < -180) { return angle + 360; }
        return angle;
    }

    public static double getAngleFromXY(double X, double Y)
    {
        return java.lang.Math.atan2(X, Y)*(180 / java.lang.Math.PI);
    }

    public static double[] getXYFromAngle(double angle)
    {
        // deg to rad
        angle /= (180 / java.lang.Math.PI);

        //rad to X,Y
        double[] XY = new double[2];
        XY[0] = java.lang.Math.sin(angle);
        XY[1] = java.lang.Math.cos(angle);
        double total = java.lang.Math.abs(XY[0]) + java.lang.Math.abs(XY[1]);
        XY[0] /= total;
        XY[1] /= total;

        return XY;
    }
}
