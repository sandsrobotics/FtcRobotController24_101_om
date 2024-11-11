package org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTicket;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import java.util.Locale;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Pinpoint extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometery Computer
    double oldTime = 0;
    PositionTracker positionTracker;
    Robot robot;

    public Pinpoint(Robot parent) {
        super(parent, "tag");
        robot = parent;
    }

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
        telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);
        telemetry.update();
        positionTracker.addPositionTicket(Pinpoint.class, new PositionTicket(posToVector(pos)));
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
        odo = robot.opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        //odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(-87.0, -5.0);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    @Override
    public void onStart() {
        positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false);
    }

    @Override
    public void onStop() {}

    private Vector3 posToVector(Pose2D pos) {
        return (new Vector3(pos.getX(DistanceUnit.INCH),pos.getY(DistanceUnit.INCH),pos.getHeading(AngleUnit.DEGREES)));
    }
}