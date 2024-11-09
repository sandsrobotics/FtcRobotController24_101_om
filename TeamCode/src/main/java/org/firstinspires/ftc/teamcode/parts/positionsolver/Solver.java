package org.firstinspires.ftc.teamcode.parts.positionsolver;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.SolverSettings;

import java.util.function.Consumer;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.ezftc.core.part.Part;
import om.self.ezftc.utils.PID;

public abstract class Solver<PARENT extends Part<?,?,?>, CONTROL> extends Part<PARENT, SolverSettings, ObjectUtils.Null> { //TODO make settings generic
    public final class Events{
        public static final String complete = "COMPLETE";
        public static final String timedOut = "TIMEOUT";
    }

    public final class Controllers{
        public final String driveController = getName() + " controller";
    }

    public final Controllers controllers;

    private double target;

    private int timesInTolerance;
    private long startTime;

    private boolean successful;

    public final PID pid = new PID();

    public Solver(PARENT parent, String name) {
        super(parent, name);
        controllers = new Controllers();
        //add event things
        getEventManager().attachToEvent(Events.complete, "set vars", () -> {
            successful = true;
        });

        getEventManager().attachToEvent(Events.timedOut, "set vars and stop part", () -> {
            successful = false;
            triggerEvent(Robot.Events.STOP);
        });
    }

    @Override
    public void onSettingsUpdate(SolverSettings settings) {
        pid.PIDs = settings.PIDCoefficients;
        pid.minClamp = -settings.maxSpeed;
        pid.maxClamp = settings.maxSpeed;

        if(settings.alwaysRun)
            getEventManager().detachFromEvent(Events.complete, "stop part");
        else
            getEventManager().attachToEvent(Events.complete, "stop part", () -> getEventManager().triggerEvent(Robot.Events.STOP));
    }

    public void setMaxPower(double maxPower){
        pid.minClamp = -maxPower;
        pid.maxClamp = maxPower;
    }

    public abstract double getError(double target);

    public abstract void move(CONTROL base);

    public abstract ControllablePart<?,?,?, CONTROL> getControlled();

    public boolean isSuccessful(){
        return successful;
    }

    public boolean isDone(){
        return successful || !isRunning();
    }

    public void setNewTarget(double target, boolean resetPID){
        this.target = target;
        reset(resetPID);
    }

    public void reset(boolean resetPID){
        if(resetPID) pid.resetErrors();
        timesInTolerance = 0;
        startTime = System.currentTimeMillis();
        successful = false;
    }

    public void reset(){
        reset(true);
    }

    @Override
    public void onBeanLoad() {

    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStart() {
        reset();
        getControlled().addController(controllers.driveController, getRun());
    }

    private Consumer<CONTROL> getRun(){
        if(getSettings().maxRuntime < 0 || getSettings().alwaysRun)
            return (base) -> {
                double error = getError(target);
                pid.updatePID(error);

                if(Math.abs(error) <= getSettings().tolerance) {
                    timesInTolerance ++;
                    if(timesInTolerance == getSettings().reqTimesInTolerance)
                        triggerEvent(Events.complete);
                }
                else
                    timesInTolerance = 0;

                move(base);
            };

        return (base) -> {
            double error = getError(target);
            pid.updatePID(error);

            if(Math.abs(error) <= getSettings().tolerance) {
                timesInTolerance ++;
                if(timesInTolerance == getSettings().reqTimesInTolerance)
                    triggerEvent(Events.complete);
            }
            else
                timesInTolerance = 0;

            if(System.currentTimeMillis() - startTime >= getSettings().maxRuntime)
                triggerEvent(Events.timedOut);

            move(base);
        };
    }

    @Override
    public void onStop() {
        getControlled().removeController(controllers.driveController);
    }
}
