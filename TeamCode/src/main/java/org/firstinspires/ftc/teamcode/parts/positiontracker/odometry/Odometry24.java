package org.firstinspires.ftc.teamcode.parts.positiontracker.odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTicket;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.AngleMath;
import om.self.ezftc.utils.Vector3;

public class Odometry24 extends LoopedPartImpl<PositionTracker, OdometrySettings, OdometryHardware> {
    int lastLeftYPos, lastRightYPos, lastXPos;
    double cumulativeDistance = 0;

    double odoAngle = 0;
    double cumulativeOdoAngle = 0; //TESTING
    double lastImuAngle =  0;

    public Odometry24(PositionTracker parent, OdometrySettings settings, OdometryHardware hardware) {
        super(parent, "odometry24");
        setConfig(settings, hardware);
    }

    public Odometry24(PositionTracker parent) {
        super(parent, "odometry24");
        //TODO make default
        setConfig(OdometrySettings.makeForOdoBot(), OdometryHardware.makeForOdoBot(parent.parent.opMode.hardwareMap));
    }

    private double getAngleFromDiff(int leftYDiff, int rightYDiff){
        return (leftYDiff - rightYDiff) * 360 / getSettings().ticksPerRotation;
    }

    @Override
    public void onRun() {

        parent.parent.opMode.telemetry.addData("y odo dist", (getHardware().leftYWheel.getCurrentPosition() + getHardware().rightYWheel.getCurrentPosition()) / 2.0);
        parent.parent.opMode.telemetry.addData("cumulativeDistance", cumulativeDistance);

        int currLeftY = getHardware().leftYWheel.getCurrentPosition();
        int currRightY = getHardware().rightYWheel.getCurrentPosition();
        int currX = getHardware().XWheel.getCurrentPosition();

        parent.parent.opMode.telemetry.addData("left y", currLeftY);
        parent.parent.opMode.telemetry.addData("right y", currRightY);
        parent.parent.opMode.telemetry.addData("middle x", currX);

        int leftYDiff = currLeftY - lastLeftYPos;
        int rightYDiff = currRightY - lastRightYPos;
        int XDiff = currX - lastXPos;

        parent.parent.opMode.telemetry.addData("left y diff", leftYDiff);
        parent.parent.opMode.telemetry.addData("right y diff", rightYDiff);
        parent.parent.opMode.telemetry.addData("x", XDiff);

        Vector3 pos = parent.getCurrentPosition();

        odoAngle = AngleMath.scaleAngle(getAngleFromDiff(leftYDiff, rightYDiff) + odoAngle);
        cumulativeOdoAngle = AngleMath.scaleAngle(getAngleFromDiff(leftYDiff, rightYDiff) + cumulativeOdoAngle);

        double imuAng = parent.getImuAngle();
        boolean imuAccurate = Math.abs(imuAng - lastImuAngle) < 0.5;
        if(imuAccurate){
            odoAngle = imuAng;
        }

        lastImuAngle = imuAng;

        parent.parent.opMode.telemetry.addData("imu accurate", imuAccurate);
        parent.parent.opMode.telemetry.addData("odo angle", odoAngle);
        parent.parent.opMode.telemetry.addData("cumulitave angle", cumulativeOdoAngle);

        double angle = odoAngle;

        double XMove = XDiff / getSettings().ticksPerInch;
        double YMove = (leftYDiff + rightYDiff) / (2 * getSettings().ticksPerInch);

        cumulativeDistance += YMove;

        //parent.addPositionTicket(Odometry.class, new PositionTicket(VectorMath.translateAsVector2(pos.withZ(imuAng), XMove, YMove), new Vector2(XMove, YMove)));
        // abandon all this and loop in some kludge code
        lkLoop();

        lastLeftYPos = currLeftY;
        lastRightYPos = currRightY;
        lastXPos = currX;
    }

    @Override
    public void onBeanLoad() {

    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStart() {
        getHardware().XWheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().leftYWheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().rightYWheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // run without encoder needed so it doesn't break the robot lift/sweep
        getHardware().XWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        getHardware().leftYWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        getHardware().rightYWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lastXPos = getHardware().XWheel.getCurrentPosition();
        lastLeftYPos = getHardware().leftYWheel.getCurrentPosition();
        lastRightYPos = getHardware().rightYWheel.getCurrentPosition();

        odoAngle = parent.getCurrentPosition().Z;
        lastImuAngle = parent.getImuAngle();
        lkOnStart();   //LK
    }

    @Override
    public void onStop() {
    }

    public void lower(){
        getHardware().leftYServo.setPosition(getSettings().leftYServoDown);
        getHardware().rightYServo.setPosition(getSettings().rightYServoDown);
        getHardware().XWheelServo.setPosition(getSettings().XServoDown);
    }

    public void raise(){
        getHardware().leftYServo.setPosition(getSettings().leftYServoUp);
        getHardware().rightYServo.setPosition(getSettings().rightYServoUp);
        getHardware().XWheelServo.setPosition(getSettings().XServoUp);
    }

    // LK Kludge Patch

    public long lkEncoderY, lkEncoderXL, lkEncoderXR;
    long lkEncoderY0, lkEncoderXL0, lkEncoderXR0;
    double lkOdoHeading, lkOdoHeading0;
    double lkImuHeading, lkImuHeading0;
    public double lkGlobalHeading, lkGlobalHeading0;
    public double lkYPos, lkXPos;
    public boolean lkUseFusedHeading = false;

    //Vector3 lkOdoRobotOffset = new Vector3 (1.75,0.25,0);          // map odo to robot (so it holds turn position better)
    Vector3 lkOdoRobotOffset = new Vector3 (1.625,0,0);          // map odo to robot (so it holds turn position better)
    //Vector3 lkOdoFieldStart = new Vector3 (-36,63,-90);  // field start position [blue right slot]
    //Vector3 lkOdoFieldStart = parent.lkStartPosition;                   //?? This value probably not ready when constructed; linked?
    Vector3 lkOdoFieldStart = new Vector3 (0,0,0);

    Vector3 lkOdoRawPose = new Vector3 (0,0,0);                 // original calculation of position before transforms applied
    Vector3 lkOdoRobotPose = new Vector3 ();                             // odo mapped to robot position (minor change)
    Vector3 lkOdoFinalPose = new Vector3 ();                             // odo mapped to field
    Vector3 lkOdoFieldOffset = new Vector3 ();                           // transform from initial position (more relevant for slamra!)


    void lkOnStart() {
        lkEncoderY0 = getHardware().XWheel.getCurrentPosition();
        lkEncoderXL0 = getHardware().leftYWheel.getCurrentPosition();
        lkEncoderXR0 = -getHardware().rightYWheel.getCurrentPosition();
        lkImuHeading0 = parent.lkRawImuAngle;
        lkOdoHeading0 = lkGetOdoHeading();
        lkGlobalHeading0 = lkImuHeading0;

        // odo start position is 0,0,0; imu should also read 0.  odoRawPose is already 0,0,0
        lkUpdateOdoRobotPose();
        lkSetOdoFinalPose();
        lkOdoFieldStart = parent.lkStartPosition;
        lkSetOdoFieldOffset();
    }

    void lkLoop() {

        /* Update encoder readings */
        lkEncoderY = getHardware().XWheel.getCurrentPosition();
        lkEncoderXL = getHardware().leftYWheel.getCurrentPosition();
        lkEncoderXR = -getHardware().rightYWheel.getCurrentPosition();

        /* Update heading */
        lkImuHeading = parent.lkRawImuAngle;  //.returnImuHeading();
        lkOdoHeading = lkGetOdoHeading();
        lkGlobalHeading = lkFusedHeading();

        /* Calculate position */
        lkUpdateXY();
        lkOdoRawPose = new Vector3(lkXPos, lkYPos, lkGlobalHeading);
        lkUpdateOdoRobotPose();
        lkSetOdoFinalPose();

        /* Write a lot of debugging to telemetry */
        parent.parent.opMode.telemetry.addData ("_lkOdoRobotOffset", toString(lkOdoRobotOffset,2));
        parent.parent.opMode.telemetry.addData ("_lkOdoFieldStart ", toString(lkOdoFieldStart,2));
        parent.parent.opMode.telemetry.addData ("_lkOdoFieldOffset", toString(lkOdoFieldOffset,2));
        parent.parent.opMode.telemetry.addData ("_lkOdoRawPose    ", toString(lkOdoRawPose,2));
        parent.parent.opMode.telemetry.addData ("_lkOdoRobotPose  ", toString(lkOdoRobotPose,2));
        parent.parent.opMode.telemetry.addData ("_lkOdoFinalPose  ", toString(lkOdoFinalPose,2));
        parent.parent.opMode.telemetry.addData ("_lkImuHeading    ", JavaUtil.formatNumber(lkImuHeading,2));
        parent.parent.opMode.telemetry.addData ("_lkOdoHeading    ", JavaUtil.formatNumber(lkOdoHeading,2));

        /* Update robot position */
        //robotPosition.X = odoFinalPose.X;   // done this way to not break the link back to Navigator class
        //robotPosition.Y = odoFinalPose.Y;   // (creating a new object messes things up)
        //robotPosition.R = odoFinalPose.R;
        // slamra example: parent.addPositionTicket(Slamra.class, new PositionTicket(slamraFinal));
        parent.addPositionTicket(Odometry24.class, new PositionTicket(lkOdoFinalPose));

    }

    /* Get XY position data from odometry wheels */
    private void lkUpdateXY () {
        // this function could use some cleanup!
        double deltaEncX, deltaEncY;
        double myHeading;

        //deltaEncX = (encoderXL - encoderXL0) / eTicksPerInch;  //for single encoder; no heading would be possible
        deltaEncX = (lkEncoderXL + lkEncoderXR - lkEncoderXL0 - lkEncoderXR0) / 2.0 / getSettings().ticksPerInch;
        deltaEncY = (lkEncoderY - lkEncoderY0) / getSettings().ticksPerInch;

        myHeading = lkGetAvgHeading(lkGlobalHeading0, lkGlobalHeading);

        parent.parent.opMode.telemetry.addData ("LK My Average Heading", myHeading);

        lkXPos = lkXPos + deltaEncX * Math.cos(Math.toRadians(myHeading));
        lkYPos = lkYPos + deltaEncX * Math.sin(Math.toRadians(myHeading));

        lkXPos = lkXPos + deltaEncY * Math.sin(Math.toRadians(myHeading));
        lkYPos = lkYPos - deltaEncY * Math.cos(Math.toRadians(myHeading));

        /* Store current values for next loop */
        lkEncoderXL0 = lkEncoderXL;
        lkEncoderY0 = lkEncoderY;
        lkEncoderXR0 = lkEncoderXR;
        lkImuHeading0 = lkImuHeading;
        lkOdoHeading0 = lkOdoHeading;
        lkGlobalHeading0 = lkGlobalHeading;
    }

    /* Calculate average of two headings */
    public double lkGetAvgHeading (double firstHeading, double secondHeading) {
        double robotHeading;
        /* Find the difference between them; based on sampling rate, assume large values wrapped */
        robotHeading = AngleMath.scaleAngle(secondHeading - firstHeading);
        robotHeading /= 2;
        robotHeading += firstHeading;
        return AngleMath.scaleAngle(robotHeading);
    }

    private double lkFusedHeading() {
        /* Don't fuse if the flag isn't set */
        if (!lkUseFusedHeading) return lkImuHeading;
        /* Use imuHeading only if it's settled */
        if (Math.abs(AngleMath.scaleAngle(lkImuHeading - lkImuHeading0)) < 0.5) return lkImuHeading;
        /* Otherwise fuse it with odoHeading data */
        return AngleMath.scaleAngle(lkGlobalHeading0 + (lkOdoHeading - lkOdoHeading0));
    }

    /* Get heading from the odometry ... accuracy varies :-(  */
    private double lkGetOdoHeading() {
        double diffX;
        diffX = lkEncoderXR - lkEncoderXL;
        diffX = diffX % getSettings().ticksPerRotation;
        diffX = diffX / getSettings().ticksPerRotation * 360;
        return AngleMath.scaleAngle(diffX);
    }

    void lkUpdateOdoRobotPose() {
        //pos1 = odoRawPose, pos2 = odoRobotOffset
        lkOdoRobotPose = lkTransformPosition(lkOdoRawPose, lkOdoRobotOffset);
    }

    void lkSetOdoFinalPose() {
        //pos1 = odoFieldOffset, pos2 = odoRobotPose
        lkOdoFinalPose = lkTransformPosition(lkOdoFieldOffset, lkOdoRobotPose);
    }

    void lkSetOdoFieldOffset() {
        Vector3 fS = lkOdoFieldStart;
        Vector3 rP = lkOdoRobotPose;
        double offsetR = fS.Z - rP.Z;
        lkOdoFieldOffset = new Vector3 (
                (fS.X - (rP.X*Math.cos(Math.toRadians(offsetR)) - rP.Y*Math.sin(Math.toRadians(offsetR)))),
                (fS.Y - (rP.X*Math.sin(Math.toRadians(offsetR)) + rP.Y*Math.cos(Math.toRadians(offsetR)))),
                (offsetR)
        );
    }

    Vector3 lkTransformPosition(Vector3 pos1, Vector3 pos2) {
        return new Vector3(
                (pos1.X + (pos2.X*Math.cos(Math.toRadians(pos1.Z)) - pos2.Y*Math.sin(Math.toRadians(pos1.Z)))),
                (pos1.Y + (pos2.X*Math.sin(Math.toRadians(pos1.Z)) + pos2.Y*Math.cos(Math.toRadians(pos1.Z)))),
                (pos1.Z + pos2.Z)
        );
    }

    public String toString(Vector3 pos, int decimals){
        return "{X=" + String.format("%."+ decimals +"f", pos.X) + ", Y=" + String.format("%."+ decimals +"f", pos.Y) + ", Z=" + String.format("%."+ decimals +"f", pos.Z) + "}";
    }

}