package org.firstinspires.ftc.teamcode.parts.positionsolver;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.Part;
import om.self.ezftc.utils.AngleMath;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.TaskEx;

public class PositionSolver extends Part<Drive, PositionSolverSettings, ObjectUtils.Null> {
//    public static final class Events{
//        public static final String done = "DONE";
//    } //TODO add done event

    private boolean t = false;
    private long startTime;

    protected PositionTracker positionTracker;

    public final ChannelSolver xChannel = new ChannelSolver(this, "x channel") {
        @Override
        public double getError(double target) {
            return target - positionTracker.getCurrentPosition().X;
        }

        @Override
        public void move(DriveControl base) {
            double r = Math.toRadians(positionTracker.getCurrentPosition().Z);
            base.power = base.power.addY(pid.returnValue() * Math.cos(r)).addX(pid.returnValue() * Math.sin(r));

//            parent.parent.parent.opMode.telemetry.addData("x pid", pid.returnValue());
//            parent.parent.parent.opMode.telemetry.addData("pid coeff", pid.PIDs);
        }
    };

    public final ChannelSolver yChannel = new ChannelSolver(this, "y channel") {
        @Override
        public double getError(double target) {
            return target - positionTracker.getCurrentPosition().Y;
        }

        @Override
        public void move(DriveControl base) {
            double r = Math.toRadians(positionTracker.getCurrentPosition().Z);
            base.power = base.power.addX(-pid.returnValue() * Math.cos(r)).addY(+pid.returnValue() * Math.sin(r));
        }
    };

    public final ChannelSolver rChannel = new ChannelSolver(this, "r channel") {
        @Override
        public double getError(double target) {
            return AngleMath.findAngleError(positionTracker.getCurrentPosition().Z, target);
        }

        @Override
        public void move(DriveControl base) {
            base.power = base.power.withZ(pid.returnValue());
        }
    };

    public PositionSolver(Drive parent) {
        super(parent, "position solver");
        setSettings(PositionSolverSettings.defaultSettings);
    }

    public PositionSolver(Drive parent, PositionSolverSettings settings){
        super(parent, "position solver");
        setSettings(settings);
    }

    public void setNewTarget(Vector3 target, boolean resetPID){
        xChannel.setNewTarget(target.X, resetPID);
        yChannel.setNewTarget(target.Y, resetPID);
        rChannel.setNewTarget(target.Z, resetPID);

        triggerEvent(Robot.Events.START);// TODO make this better
    }

    public void setMaxPower(double maxX, double maxY, double maxR){
        xChannel.setMaxPower(maxX);
        yChannel.setMaxPower(maxY);
        rChannel.setMaxPower(maxR);
    }

    public boolean isDone(){
        return xChannel.isDone() && yChannel.isDone() && rChannel.isDone();
    }

    public void addMoveToTaskEx(Vector3 target, TaskEx task){
        task.addStep(() -> setNewTarget(target, true));
        task.addStep(this::isDone);
    }

    public void addMoveToTaskExNoWait(Vector3 target, TaskEx task){
        task.addStep(() -> setNewTarget(target, true));
    }

    public void addMoveToTaskEx(Vector3 target, TaskEx task, int time){
        //sanitize input
        if(time <= 0) return;
        task.addStep(() -> {startTime = System.currentTimeMillis();});
        task.addStep(() -> setNewTarget(target, true));
        task.addStep(() -> (System.currentTimeMillis() - startTime >= time) || isDone());
    }

    private void addMoveToTaskEx(Vector3 target, TaskEx task, int time, boolean wait, PositionSolverSettings psSetting) {
        //sanitize input
        if(time < 0) return;
        task.addStep(() -> setSettings(psSetting));
        if (!wait) {
            addMoveToTaskExNoWait(target, task);
        } else if (time==0) {
            addMoveToTaskEx(target, task);
        } else {
            addMoveToTaskEx(target, task, time);
        }
    }

    @Override
    public void onBeanLoad() {
        positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false, false);
    }

    @Override
    public void onSettingsUpdate(PositionSolverSettings settings) {
        xChannel.setSettings(settings.xChannelSettings);
        yChannel.setSettings(settings.yChannelSettings);
        rChannel.setSettings(settings.rChannelSettings);
    }

    @Override
    public void onInit() {}

    @Override
    public void onStart() {
        if(!t) {
            t = true;
            setNewTarget(positionTracker.getCurrentPosition(), true);
        }
    }

    @Override
    public void onStop() {

    }
}
