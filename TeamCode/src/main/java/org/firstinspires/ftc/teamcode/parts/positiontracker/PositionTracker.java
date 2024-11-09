package org.firstinspires.ftc.teamcode.parts.positiontracker;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.slamra.Slamra;

import java.util.Hashtable;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.AngleMath;
import om.self.ezftc.utils.Vector2;
import om.self.ezftc.utils.Vector3;
import om.self.ezftc.utils.VectorMath;


public class PositionTracker extends LoopedPartImpl<Robot, PositionTrackerSettings, PositionTrackerHardware> {
    private Vector3 currentPosition = new Vector3();
    private Vector2 relativePosition = new Vector2();
    private double offset;
    private long lastUpdateTime = System.currentTimeMillis();
    public Class positionSourceId; //TODO make better
    private Hashtable<Class, PositionTicket> tickets = new Hashtable();
    private double imuAngle = 0;
    public PositionTicket tagTicket;

    public double lkRawImuAngle = 0; // LK
    public Vector3 lkStartPosition; // LK

    public PositionTracker(Robot robot) {
        super(robot, "position tracker", robot.startTaskManager);
        setConfig(PositionTrackerSettings.makeDefault(), PositionTrackerHardware.makeDefault(robot));
    }

    public PositionTracker(Robot robot, PositionTrackerSettings positionTrackerSettings, PositionTrackerHardware positionTrackerHardware) {
        super(robot, "position tracker", robot.startTaskManager);
        setConfig(positionTrackerSettings, positionTrackerHardware);
    }

    public void setAngle(double angle){
        updateAngle();
        offset += currentPosition.Z - angle;
        imuAngle = angle;
        currentPosition = currentPosition.withZ(angle);
    }

    public Vector3 getCurrentPosition() {
        return currentPosition;
    }

    /**
     * @param currentPosition the current position
     * @deprecated you should add a position ticket so it integrates!!
     */
    @Deprecated
    public void setCurrentPosition(Vector3 currentPosition) {
        this.currentPosition = currentPosition;
        lastUpdateTime = System.currentTimeMillis();
    }

    public Vector2 getRelativePosition() {
        return relativePosition;
    }

    public void addPositionTicket(Class id, PositionTicket pt){
        tickets.put(id, pt);
    }

    public boolean isPositionStale(){
        return System.currentTimeMillis() - lastUpdateTime > getSettings().stalePosTime;
    }

    public double getImuAngle(){
        return imuAngle;
    }

    private void updateAngle() {
        if(getHardware() != null) {
            double angle = getHardware().imu.getAngularOrientation(AxesReference.EXTRINSIC, getSettings().axesOrder, AngleUnit.DEGREES).thirdAngle;
            lkRawImuAngle = AngleMath.scaleAngle(angle); //LK
            if (getSettings().flipAngle)
                angle *= -1;
            angle -= offset;
            imuAngle = AngleMath.scaleAngle(angle);
            setCurrentPosition(currentPosition.withZ(imuAngle));
            PositionTicket currentPosTix = tickets.get(positionSourceId);
            if (currentPosTix != null) {
                Vector3 currentPos = currentPosTix.position;
                PositionTicket NewCurrentPosTix = new PositionTicket(currentPos.withZ(imuAngle));
                tickets.put(positionSourceId, NewCurrentPosTix);
            }
        }
    }

    @Override
    public void onBeanLoad() {
        if(getBeanManager().getBestMatch(Slamra.class, true, true) != null)
            positionSourceId = Slamra.class;
        else if(getBeanManager().getBestMatch(EncoderTracker.class, true, true) != null)
            positionSourceId = EncoderTracker.class;
        else if(getBeanManager().getBestMatch(Odometry.class, true, true) != null)
            positionSourceId = Odometry.class;

        //TODO something better
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStart() {
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void onSettingsUpdate(PositionTrackerSettings positionTrackerSettings) {
        setAngle(positionTrackerSettings.startPosition.Z);
        currentPosition = positionTrackerSettings.startPosition;
        lkStartPosition = positionTrackerSettings.startPosition; // LK
    }

    @Override
    public void onHardwareUpdate(PositionTrackerHardware hardware) {
        hardware.imu.initialize(hardware.parameters);

        while (!hardware.imu.isGyroCalibrated())
        {
            parent.opMode.telemetry.addData("gyro status", "calibrating");
            parent.opMode.telemetry.update();
        }

        parent.opMode.telemetry.addData("gyro status", "calibrated :)");
        parent.opMode.telemetry.update();
    }

    @Override
    public void onRun() {
        updateAngle();

        if(positionSourceId != null && tickets.containsKey(positionSourceId)){
            lastUpdateTime = System.currentTimeMillis();
            PositionTicket ticket = tickets.get(positionSourceId);
            tagTicket = tickets.get(AprilTag.class);
            currentPosition = ticket.position; //todo add something better
            if(tagTicket != null)
                currentPosition = tagTicket.position;
            relativePosition = VectorMath.add(relativePosition, ticket.robotRelative);
            tickets.clear();
        }
    }

    @Override
    public void onStop() {

    }

}