package om.self.ezftc.utils;


import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Vector3 {
	public final double X,Y,Z;

	/**
	 * hello this is a comment
	 * @param X
	 * @param Y
	 * @param Z
	 */
	public Vector3(double X, double Y, double Z){
		this.X = X;
		this.Y = Y;
		this.Z = Z;
	}

	public Vector3(double[] vals){
		this.X = vals[0];
		this.Y = vals[1];
		this.Z = vals[2];
	}

	public Vector3(){
		X = 0;
		Y = 0;
		Z = 0;
	}

	public Vector3(double val){
		this.X = val;
		this.Y = val;
		this.Z = val;
	}

	public double[] toArray() {
		return new double[]{X,Y,Z};
	}

	public double get(int index){
		switch(index){
			case 0: return X;
			case 1: return Y;
			case 2: return Z;
			default: throw new IndexOutOfBoundsException("the index '" + index + "' is not inside a vector 3");
		}
	}

	public Vector3 switchXY(){
		return new Vector3(Y, X, Z);
	}

	public Vector3 switchXZ(){
		return new Vector3(Z, Y, X);
	}

	public Vector3 switchYZ(){
		return new Vector3(X, Z, Y);
	}

	public Vector3 invert(){
		return new Vector3(-Y, -X, -Z);
	}

	public Vector3 invertX(){
		return new Vector3(-X, Y, Z);
	}

	public Vector3 invertY(){
		return new Vector3(X, -Y, Z);
	}

	public Vector3 invertZ(){
		return new Vector3(X, Y, -Z);
	}

	public Vector3 withX(double X){return new Vector3(X,Y,Z);}

	public Vector3 addX(double X){return new Vector3(this.X + X,Y,Z);}

	public Vector3 withY(double Y){return new Vector3(X,Y,Z);}

	public Vector3 addY(double Y){return new Vector3(X, this.Y + Y,Z);}

	public Vector3 withZ(double Z){return new Vector3(X,Y,Z);}

	public Vector3 addZ(double Z){return new Vector3(X,Y,this.Z + Z);}

	public Pose2d toPose2d(){
		return new Pose2d(X, Y, Math.toRadians(Z));
	}

	public boolean inTolerance(Vector3 targetPos, Vector3 tol){
		return VectorMath.inTolerance(this, targetPos, tol);
	}

	public boolean inTolerance(double[] targetPos, double[] tol){
		return VectorMath.inTolerance(toArray(), targetPos, tol);
	}

	@Override
	public boolean equals(Object obj) {
		if(! (obj instanceof Vector3)) return false;
		Vector3 other = (Vector3) obj;
		return other.X == X && other.Y == Y && other.Z == Z;
	}

	public String toString(int decimals){
		return "Vector3{X=" + String.format("%."+ decimals +"f", X) + ", Y=" + String.format("%."+ decimals +"f", Y) + ", Z=" + String.format("%."+ decimals +"f", Z) + "}";
	}

	@Override
	public String toString() {
		return toString(2);
	}
}