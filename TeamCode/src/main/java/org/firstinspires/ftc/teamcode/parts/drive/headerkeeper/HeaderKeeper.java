package org.firstinspires.ftc.teamcode.parts.drive.headerkeeper;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.part.Part;
import om.self.ezftc.utils.AngleMath;
import om.self.ezftc.utils.PID;
import om.self.supplier.modifiers.EdgeExModifier;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class HeaderKeeper extends Part<Drive, HeaderKeeperSettings, ObjectUtils.Null> {
    private PositionTracker pt;
    private HeaderKeeperSettings settings;

    private double lastHeading;
    private final PID pid = new PID();

    private final TimedTask headerKeeper = new TimedTask("keep header");
    private final EdgeExModifier edgeModifier = new EdgeExModifier();

    public HeaderKeeper(Drive parent, HeaderKeeperSettings settings) {
        super(parent, "header keeper");
        setSettings(settings);
    }

    public HeaderKeeper(Drive parent) {
        super(parent, "header keeper");
        setSettings(HeaderKeeperSettings.makeDefault());
    }

    public HeaderKeeperSettings getSettings() {
        return settings;
    }

    public void setSettings(HeaderKeeperSettings settings) {
        this.settings = settings;

        pid.PIDs = settings.pidCoefficients;
        pid.minClamp = -settings.maxI;
        pid.maxClamp = settings.maxI;
        buildHeaderKeeper();
    }

    @Override
    public void onBeanLoad() {
        pt = getBeanManager().getBestMatch(PositionTracker.class, false, false);
    }

    private void buildHeaderKeeper(){
        headerKeeper.clear();
        headerKeeper.addDelay(getSettings().headingSettleDelay);
        headerKeeper.addStep(() -> {
            lastHeading = pt.getCurrentPosition().Z;
            pid.resetErrors();
        });
        headerKeeper.addStep(() -> pid.updatePID(AngleMath.findAngleError(pt.getCurrentPosition().Z, lastHeading)), () -> false);
    }

    @Override
    public void onInit() {
        //create header keeper task
        headerKeeper.attachParent(getTaskManager());
        buildHeaderKeeper();

        //automatically trigger and pause
        edgeModifier.setOnRise(headerKeeper::restart);
        edgeModifier.setOnFall(() -> headerKeeper.runCommand(Group.Command.PAUSE));
    }

    @Override
    public void onStart() {
        parent.addController("header keeper", (control) -> {
            if(edgeModifier.apply(Math.abs(control.power.Z) < settings.minRegisterVal)){
                control.power = control.power.withZ(pid.returnValue());
            }
        });
    }

    @Override
    public void onStop() {
        parent.removeController("header keeper");
    }
}
