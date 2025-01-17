package org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTicket;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import java.util.Locale;
import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Pinpoint extends LoopedPartImpl<PositionTracker, ObjectUtils.Null, ObjectUtils.Null> {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometery Computer
    double oldTime = 0;
    Boolean reset = true;

    String hwDeviceName = "odo";
    double settingsXoffset = -80.0;
    double settingsYoffset = 44.0;
    float settingsResolution = 19.89436789f; //19.89436789f for 4-bar;  13.26291192f for swingarm
    GoBildaPinpointDriver.EncoderDirection settingsXdirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    GoBildaPinpointDriver.EncoderDirection settingsYdirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    //    goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
    //    goBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod

        /* For flipbot:
        odo = new Pinpoint(pt, true, -56.0, 52.0, 13.26291192f,
                GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        */

    public Pinpoint(PositionTracker parent, Boolean reset) {
        super(parent, "Pinpoint");
        this.reset = reset;
    }

    public Pinpoint(PositionTracker parent) {
        super(parent, "Pinpoint");
    }

    public Pinpoint(PositionTracker parent,
                    Boolean reset,
                    String hwDeviceName,
                    double settingsXoffset,
                    double settingsYoffset,
                    float settingsResolution,
                    GoBildaPinpointDriver.EncoderDirection settingsXdirection,
                    GoBildaPinpointDriver.EncoderDirection settingsYdirection) {
        super(parent, "Pinpoint");
        this.reset = reset;
        this.hwDeviceName = hwDeviceName;
        this.settingsXoffset = settingsXoffset;
        this.settingsYoffset = settingsYoffset;
        this.settingsResolution = settingsResolution;
        this.settingsXdirection = settingsXdirection;
        this.settingsYdirection = settingsYdirection;
    }

    public void setPosition(Vector3 vector) {
        odo.setPosition(vector3ToPose2D(vector));
    }
    public Vector3 getPosition() {return posToVector3(odo.getPosition());}

    @Override
    public void onRun() {
        /*
        Request an update from the Pinpoint odometrey computer. This checks almost all outputs
        from the device in a single I2C read.
         */
        odo.update();

            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
        Pose2D pos = odo.getPosition();

        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        parent.parent.opMode.telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));

        GoBildaPinpointDriver.DeviceStatus status = odo.getDeviceStatus();
            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */

        //lk: don't set the position ticket if the pinpoint isn't READY and returning valid data
        //todo: if pinpoint isn't working, fall back to some other kind of navigation (e.g., motor encoders)
        if (status==GoBildaPinpointDriver.DeviceStatus.READY) {
            parent.addPositionTicket(Pinpoint.class, new PositionTicket(posToVector3(pos)));
        }
    }

    @Override
    public void onBeanLoad() {
    }

    /**
     * WARNING: beans may not be loaded onInit, please use onStart for beans
     */
    @Override
    public void onInit() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        odo = parent.parent.opMode.hardwareMap.get(GoBildaPinpointDriver.class, hwDeviceName);
        //odo = parent.parent.opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        //odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        //odo.setOffsets(-80.0, 44.0);
        odo.setOffsets(settingsXoffset, settingsYoffset);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        //odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderResolution(settingsResolution);
        //odo.setEncoderResolution(13.26291192);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        //odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderDirections(settingsXdirection, settingsYdirection);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        if (reset) odo.resetPosAndIMU();
        //odo.setPosition(vector3ToPose2D(parent.getCurrentPosition()));
    }

    @Override
    public void onStart() {
    }

    @Override
    public void onStop() {}

    private Vector3 posToVector3(Pose2D pos) {
        return (new Vector3(pos.getX(DistanceUnit.INCH),pos.getY(DistanceUnit.INCH),pos.getHeading(AngleUnit.DEGREES)));
    }

    private Pose2D vector3ToPose2D(Vector3 vector) {
        return(new Pose2D(DistanceUnit.INCH,vector.X,vector.Y,AngleUnit.DEGREES, vector.Z));
    }
}