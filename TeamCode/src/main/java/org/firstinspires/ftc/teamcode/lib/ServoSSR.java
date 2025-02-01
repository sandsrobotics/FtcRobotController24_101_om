package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoSSR implements Servo {

    private final Servo servo;
    private double offset = 0.0;            // offset is useful for syncing a pair of servos or "calibrating" a replacement servo
    private boolean enabled = false;        // tracks whether or not the servo is (or should be) enabled
    private boolean eStopped = true;        // tracks whether the servo is stopped in such a way that the position is unpredictable
    private boolean unknown = true;         // tracks whether the servo position is unknown (due to eStop or external shenanigans)
    private int sweepTime = 1500;           // the time (in ms) it takes the servo to move its entire range (account for loading when setting!)
    private int sweepTimeDec = 1500;        // the time (in ms) it takes the servo to move its entire range (account for loading when setting!)
    private int wakeTime = 200;             // a short interval for the servo to move from disabled/parked to last position
    private double scale = 1.0;             // scale is potentially useful when replacing a servo with a different range (e.g., 270° vs 300°)
    private long timer = 0;                 // a clock time to track whether a move should be complete

    public ServoSSR(Servo servo) {
        this.servo = servo;
    }

    // Future improvement possibility 1: For servos with feedback (e.g., Axon Max+),
    //     add the ability to associate and configure an analog channel to read actual position and verify movement.
    // Future improvement possibility 2: Estimate servo position based on timer

    // setters

    /**
     * Sets an offset value for the servo that will be added when setting a position with setPosition()
     * @param offset the offset to added, limited to -.25 to +.25.
     * @return this for method chaining
     */
    public ServoSSR setOffset(double offset) {
        this.offset = clamp(offset, -.25, .25);
        return this;
    }

    /**
     * Sets the servo sweep time (full time to traverse from minimum to maximum position; e.g., -150° to 150°).
     * The value supplied should account for how the servo is loaded.
     * @param sweepTime the time in ms, limited to 0–5000.
     * @return this for method chaining
     */
    public ServoSSR setSweepTime(int sweepTime) {
        this.sweepTime = (int)clamp(sweepTime, 0, 5000);
        this.sweepTimeDec = (int)clamp(sweepTime, 0, 5000);
        return this;
    }
    public ServoSSR setSweepTime(long sweepTime) {
        return setSweepTime((int)sweepTime);
    }

    /**
     * Sets the servo sweep time (full time to traverse from minimum to maximum position; e.g., -150° to 150°).
     * The value supplied should account for how the servo is loaded.
     * @param sweepTimeTo1 the time in ms, going "forward" from 0 to 1, limited to 0–5000.
     * @param sweepTimeTo0 the time in ms, going "reverse" from 1 to 0, limited to 0–5000.
     * @return this for method chaining
     */
    public ServoSSR setSweepTime(int sweepTimeTo1, int sweepTimeTo0) {
        this.sweepTime = (int)clamp(sweepTimeTo1, 0, 5000);
        this.sweepTimeDec = (int)clamp(sweepTimeTo0, 0, 5000);
        return this;
    }
    public ServoSSR setSweepTime(long sweepTimeTo1, long sweepTimeTo0) {
        return setSweepTime((int)sweepTimeTo1, (int)sweepTimeTo0);
    }

    /**
     * Sets the servo "wake" time. If the servo was disabled, this time is allowed for it to return to its previous position.
     * Assumes a small amount of movement has happened while it is disabled.
     * @param wakeTime the time in ms, limited to 0–1000.
     * @return this for method chaining
     */
    public ServoSSR setWakeTime(int wakeTime) {
        this.wakeTime = (int)clamp(wakeTime,0,1000);
        return this;
    }
    public ServoSSR setWakeTime(long wakeTime) {
        return setWakeTime((int)wakeTime);
    }

    /**
     * Sets a scale value for the servo that will be used (multiplied) when setting a position with setPosition().
     * The scale will be centered around the midpoint of 0.5.
     * <P>Note: If scale > 1, the range will be clamped to the range 0–1, and the isAtPosition() and isSetPosition() checks
     * will not work correctly for the clamped positions.
     * @param scale the scale multiplier, limited to .5–1.5 (50-150%)
     * @return this for method chaining
     */
    public ServoSSR setScale(double scale) {
        this.scale = clamp(scale, 0.5, 1.5);
        return this;
    }

    /**
     * Sets the servo Pwm range to the maximum; i.e., 500-2500 μs
     * @return this for method chaining
     */
    public ServoSSR setFullPwmRange() {
        return setPwmRange(500,2500);
    }

    /**
     * Sets the servo Pwm range
     * @param low the low end of the Pwm range in μs
     * @param high the high end of the Pwn range in μs
     * @return this for method chaining
     */
    public ServoSSR setPwmRange(double low, double high) {
        double lowC = clamp(low,500,2500);
        double highC = clamp(high,500,2500);
        if (lowC > highC) {
            double temp = lowC;
            lowC = highC;
            highC = temp;
        }
        ((ServoImplEx)servo).setPwmRange(new PwmControl.PwmRange(lowC, highC));
        return this;
    }

    /**
     * Resets the internal movement timer. Potentially useful when you know the tracking timer may not be accurate (e.g., during initialization
     * or when the servo is under heavy load). With no parameter, defaults to the full sweep time.
     * <p>Note that when setting a new position, the delay will be reset with a maximum of the full sweep time, so if you want to delay isDone()
     * longer than that, you will need to use this method after setting the position.
     * @param delay the amount of time in ms to set the timer in the future (one time only). If not provided, defaults to sweep time.
     *              Limited to 0–5000.
     * @return this for method chaining
     */
    public ServoSSR resetTime(int delay) {
        timer = System.currentTimeMillis() + (long)clamp(delay,0,5000);
        return this;
    }
    public ServoSSR resetTime() {
        return resetTime(Math.max(sweepTime, sweepTimeDec));
    }

    // enabling and disabling pwm

    /**
     * Emergency Stop: Disables the Pwm signal for the servo such that the position is assumed to be lost/unknown.
     * Servo behavior may vary; goBilda servos will power down. Will automatically re-enable when another position is set.
     */
    public void stop() {
        disable();
        eStopped = true;  // we no longer know where the servo is, so need to time accordingly next move
        unknown = true;
    }

    /**
     * Disables the Pwm signal for the servo. Servo behavior may vary; goBilda servos will power down.
     * This is different than stop() in that it's assumed the servos won't move much (e.g., parked or resting).
     * Will automatically re-enable when another position is set.
     */
    public void disable() {
        //((ServoControllerEx) getController()).setServoPwmDisable(getPortNumber());
        if (!isDone()) {
            if (enabled) {       // these are separated for clarity; isDone() calls isEnabled() and potentially changes the enabled variable
                eStopped = true; // if not already disabled and in motion, assume worst and convert to estop
                unknown = true;
            }
        }
        ((ServoImplEx)servo).setPwmDisable();
        enabled = false;
        timer = 0;
    }

    /**
     * Enables the Pwm signal for the servo. (Will automatically re-enable when another position is set.)
     */
    public void enable() {
        //((ServoControllerEx) getController()).setServoPwmEnable(getPortNumber());
        ((ServoImplEx)servo).setPwmEnable();
        enabled = true;
    }

    // status responders & getters

    /**
     * Gets the stored offset value
     * @return the offset that is added when setting position
     */
    public double getOffset() {
        return offset;
    }

    /**
     * Gets the servo position last set, accounting for the offset
     * @return the servo position - offset
     */
    public double getPositionWithOffset() {
        //return clamp(getPosition() - offset);
        // reverse order... subtract the offset, then inversely scale
        return unscalePos(clamp(getPosition() - offset));
    }

    /**
     * Determine if the servo is expected to be finished moving
     * (i.e., the timer associated with the servo movement is complete and the servo is enabled).
     * <P>Note: if the servo gets disabled, this will not be true, even if it is re-enabled.
     * @return TRUE if the movement should be complete
     */
    public boolean isDone() {
//        return enabled && timer != 0 && System.currentTimeMillis() >= timer;
        return isEnabled() && timer != 0 && isTimerDone();
    }

    /**
     * Determine if the servo timer is complete and therefore it is expected to be finished moving
     * (does not account for the possibility that it was disabled)
     * @return TRUE if the timer is complete
     */
    public boolean isTimerDone() {
        return System.currentTimeMillis() >= timer;
    }

    /**
     * Gets the amount of time remaining before the servo move is expected to be complete
     * @return the time remaining in ms
     */
    public long timeRemaining() {
        return Math.max(timer - System.currentTimeMillis(), 0);
    }

    /**
     * Determine if the servo is at a position (i.e., set to a certain position and the associated timer is complete)
     * @param comparePosition the position to check against the actual set position
     * @return TRUE if both the servo is set to that position and the time is complete
     */
    public boolean isAtPosition(double comparePosition) {
        return isSetPosition(comparePosition) && isDone();
    }

    /**
     * Determine if the servo is set to a certain position (not accounting for the timer)
     * @param comparePosition the position to check against the actual set position
     * @return TRUE if the servo is set to that position
     */
    public boolean isSetPosition(double comparePosition) {
        return(Math.round(getPositionWithOffset()*100.0) == Math.round(comparePosition*100.0));  // deals with rounding error
    }
    private boolean isSetPos(double comparePosition) {
        // for internal use, where the position is already scaled and offset
        return(Math.round(getPosition()*100.0) == Math.round(comparePosition*100.0));  // deals with rounding error
    }

    /**
     * Determine if the Pwm signal is enabled for the servo
     * (as tracked internally by the wrapper and checked against actual state of the controller)
     * @return TRUE if the servo Pwm is enabled
     */
    public boolean isEnabled() {
        // does ServoControllerEx cache this, or does it have to query the hardware resulting in a time penalty?
        boolean pwmState = ((ServoImplEx)servo).isPwmEnabled();
        if (enabled && !pwmState) {   // detect pwmDisabled without using internal methods and assume the worst
            enabled=false;
            eStopped=true;
            unknown=true;
        }
        else if (!enabled && pwmState) {   // detect pwmEnabled without using internal methods
            enabled=true;
            eStopped=false;
            unknown=true;         // .getPosition() should work? But with all the trouble we've had, assume the worst.
        }
        return enabled;
    }

    /**
     * Determine if the Pwm signal is disabled for the servo (as tracked internally by the wrapper)
     * @return TRUE if the servo Pwm is disabled
     */
    public boolean isDisabled() {
//        return !enabled;
        return !isEnabled();
    }

    /**
     * Determine if the servo is in an emergency stopped state
     * (Pwm signal disabled for the servo, as tracked internally by the wrapper, and position unknown)
     * @return TRUE if the servo is stopped
     */
    public boolean isStopped() {
        isEnabled();      // added to check for external unpredictable disables that should be treated the same
        return eStopped;
    }

    /**
     * @return the servo object, for whatever reason it's needed
     */
    public Servo getServo() {
        return servo;
    }

    // Servo class overrides

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public void setPosition(double position) {
        /* 0. Scale and offset the position */
        double pos = clamp(scalePos(position) + offset);

        /* 1. Make sure the state variables are up to date */
        isEnabled();

        /* 2. Don't update the timer if enabled and the position is already set the same */
        //if (enabled && isSetPosition(position)) {
        if (enabled && isSetPos(pos)) {
            //servo.setPosition(clamp(position + offset));          // this probably isn't needed, but added while debugging undesirable behavior
            servo.setPosition(pos);          // this probably isn't needed, but added while debugging undesirable behavior
            return;                                        // has already been set (but not necessarily done moving), no need to increment timer
        }

        /* 3. Re-enable if necessary. Setting position should do this, but the very first position was observed to not work.
           This also is now necessary for the updated isEnabled() method */
        if (!enabled) enable();

        /* 4. Calculate the timer and set the position */
        //timer = calcSweepTimerValue(position);
        timer = calcSweepTimerValue(pos);
        //servo.setPosition(clamp(position + offset));
        servo.setPosition(pos);

        /* 5. Update tracking variables */
        enabled = true;                                    // setting a position re-enables, so update the trackers
        eStopped = false;
        unknown = false;
    }

    /**
     * Set the servo "power" similar to CRServo.setPower() where the power is in the range -1 to 1
     * instead of the normal servo range of 0 to 1.
     * @param power the desired "power"
     */
    public void setPower(double power) {
        //setPosition(0.5 + Math.signum(power) * Math.abs(power) / 2.0);   // why overcomplicate it?
        setPosition(clamp(0.5 * power + 0.5));
    }

    /**
     * Get the servo "power" similar to CRServo.getPower() where the power is in the range -1 to 1
     * instead of the normal servo range of 0 to 1.
     */
    public double getPower() {
        return servo.getPosition() * 2.0 - 1.0;
    }

    /**
     * If using a Blinkin LED device, sets the Blinkin to a desired pattern number.
     * It does this by converting that number to a pulse width between 1000-2000 μs, and converting that to a 0-1 servo position.
     * <P>For this to work, the servo controller must be set to use the full range of 500-2500 μs,
     * which can be set by using the .setFullPwmRange() method.
     * @param pattern the desired pattern number between 1 and 100
     */
    public void setBlinkinPattern(int pattern) {
        // Q: Why do this craziness instead of defining the servo port as a Blinkin?
        // A: Because that changes the stored config and other things. Easier to just use all the servo ports as servos.
        if (pattern<1 || pattern>100) return;              // Or throw an error? Legal Blinkin patterns are between 1 and 100
        int pulseWidth = 995 + 10*pattern;                 // Convert pattern number to pulse width between 1000-2000 μs)
        double setting = (pulseWidth - 500) / 2000.0;      // Covert pulse width to 0-1 servo position (based on 500-2500 μs)
        servo.setPosition(setting);
    }

    /**
     * If using a Blinkin LED device, sets the Blinkin to a desired pattern by name.
     * It does this by converting that name to a pulse width between 1000-2000 μs, and converting that to a 0-1 servo position.
     * <P>For this to work, the servo controller must be set to use the full range of 500-2500 μs,
     * which can be set by using the .setFullPwmRange() method.
     * @param pattern the desired pattern from enum RevBlinkinLedDriver.BlinkinPattern
     */
    public void setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        setBlinkinPattern(pattern.ordinal()+1);            // Ordinal starts at 0, so need to add 1 (convert 0-99 to 1-100)
    }

    // internal methods

    private long calcSweepTimerValue(double newPosition) {
        /* if the position is unknown, allow the maximum full sweep time */
        if (unknown) {
            return System.currentTimeMillis() + Math.max(sweepTime, sweepTimeDec);
        }
        /* select sweep time based on the direction of movement (towards 1 or towards 0) */
        int sweepTimeByDir = newPosition > getPosition() ? sweepTime : sweepTimeDec;
        /* enabled, timer not reset, timer complete = should be at last requested position */
        if (isDone()) {
            return System.currentTimeMillis() + (long)(calcSweepChange(newPosition) * sweepTimeByDir);
        }
        /* if the previous move was not complete, assume the worst case scenario for the next move
           (i.e., the rest of the time remaining from the previous move plus the new move time,
           but never more than the full sweep time)
           possible future to do: calculate the predicted position based on time and last position */
        return Math.min(System.currentTimeMillis() + sweepTimeByDir,      // need to cap this at full sweep time
                Math.max(timer, System.currentTimeMillis())               // remaining time, not less than current time (accounts for timer reset to 0)
                + (long)(calcSweepChange(newPosition) * sweepTimeByDir)   // normally calculated time for movement
                + (enabled ? 0 : wakeTime));                              // add waketime if disabled (and expected to be near last position)
    }

    private double calcSweepChange(double newPosition) {
        //return Math.abs(getPositionWithOffset()-newPosition);
        return Math.abs(getPosition()-newPosition);
    }

    private double clamp(double pos) {
        return Math.max(0, Math.min(pos, 1));
    }
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }

    private double scalePos(double pos) {
        return clamp(((clamp(pos) - 0.5) * scale) + 0.5);
    }

    private double unscalePos(double pos) {
        return clamp(((clamp(pos) - 0.5) / scale) + 0.5);
    }
}

/*
Usage
=====

Instantiate with a Servo interface instance:
	ServoSSR servoWhatever;
	servoWhatever = new ServoSSR(parts.robot.servo0);   -or similar-
	servoWhatever = new ServoSSR(hardwareMap.get(Servo.class,"servo0");

For convenience, the new settings can be chained:
    servoWhatever.setSweepTime(1000).setWakeTime(250).setOffset(-0.05).setFullPwmRange()

All of the ordinary Servo methods and properties are available with the following changes and additions:

setPosition() - sets the position with offset and calculates the expected movement time
setOffset(offset) - set an offset that will be added to positions (for tuning a replacement servo or one of a pair acting together)
setScale(scale) - set a scale multiplier for positions
setSweepTime(sweepTime) - set the time expected for the servo to move its entire range
setSweepTime(sweepTimeTo1, sweepTimeTo0) - set the bi-directional time expected for the servo to move its entire range
setWakeTime(wakeTime) - set the time expected for the servo to move back to its position after being disabled
setFullPwmRange() - sets the controller to use pwm range of 500-2500 μs vs. the default of 600-2400 μs
setPwmRange(low, high) - sets the controller to use an arbitrary pwm range
resetTime(delay) - resets the internal timer to delay or default of sweepTime
stop() - disables the servo pwm signal and its position will be unknown/unpredictable (e.g., emergency stop)
disable() - disables the servo pwm signal and assumes it will stay near its last position (e.g., docked or parked)
enable() - enables the servo pwm signal (not usually necessary to do manually)
getOffset() - returns the offset value
getPositionWithOffset() - returns the position set (subtracting the offset, so back to the original position vs. the offset position)
isDone() - is the servo done moving? (servo is enabled, timer is complete)
isTimerDone() - is the servo timer done? (does not account for the possibility that the servo has been disabled)
timeRemaining() - returns the time remaining before the servo is expected to have finished moving
isAtPosition(comparePosition) - is the servo done moving, and is the position this position?
isSetPosition(comparePosition) - is the servo set to this position? (may still be moving)
isEnabled() - is the servo enabled?
isDisabled() - is the servo disabled?
isStopped() - is the servo stopped?
getServo() - get the servo object for whatever reason
setPower(power) - sets the "power" like a CRservo, -1 to 1.
getPower() - returns the "power", -1 to 1.
setBlinkinPattern(pattern) - sets using pattern as int or RevBlinkinLedDriver.BlinkinPattern; must use setFullPwmRange()

 */