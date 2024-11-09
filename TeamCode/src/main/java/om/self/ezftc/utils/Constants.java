package om.self.ezftc.utils;


public class Constants{
    public static final float mmPerInch        = 25.4f;
    public static final float mPerInch         = 0.0254f;
    public static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    public static final float halfField        = 72 * mmPerInch;
    public static final float halfTile         = 12 * mmPerInch;
    public static final float oneAndHalfTile   = 36 * mmPerInch;
    public static final double tileSide = 23.5;

    public static Vector3 tileToInch(Vector3 p){
        return VectorMath.multiplyAsVector2(p, tileSide);
    }

    public static float[] inchesToMM(float[] arr){
        float[] out = new float[arr.length];
        for (int i = 0; i < arr.length; i++){
            out[i] = arr[i] * mmPerInch;
        }
        return out;
    }
}