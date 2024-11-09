package om.self.ezftc.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PID
{
	public PIDCoefficients PIDs;
	public double maxClamp;
	public double minClamp;

	private double value = 0;

	private double totalError = 0;
	private double lastError = 0;
	private long lastTime = System.nanoTime();

	public PID(){
		this.PIDs = new PIDCoefficients(0,0,0);
		this.minClamp = -1;
		this.maxClamp = 1;
	}

	public PID(PIDCoefficients PIDs, double minClamp, double maxClamp)
	{
		this.PIDs = PIDs;
		this.minClamp = minClamp;
		this.maxClamp = maxClamp;
	}

	public void updatePID(double error)
	{
		if(Math.signum(error) != Math.signum(totalError)) totalError = 0;

		totalError += error;

		double calculatedI = (totalError * PIDs.i);
		if(calculatedI > maxClamp){
			totalError = maxClamp/PIDs.i;
			calculatedI = maxClamp;
		}
		else if(calculatedI < minClamp){
			totalError = minClamp/ PIDs.i;
			calculatedI = minClamp;
		}

		double calculatedD = (((error - lastError) * PIDs.d) / ((double) (System.nanoTime() - lastTime) / 1000000000.0));

		value = (error * PIDs.p) + calculatedI - calculatedD;

		lastTime = System.nanoTime();
		lastError = error;
	}

	public void resetErrors()
	{
		totalError = 0;
		lastError = 0;
		lastTime = System.nanoTime();
	}

	public double updatePIDAndReturnValue(double error)
	{
		updatePID(error);
		return returnValue();
	}

	public double returnValue()
	{
		return Math.min(Math.max(value, minClamp), maxClamp);
	}

	public double returnUncappedValue()
	{
		return value;
	}
}