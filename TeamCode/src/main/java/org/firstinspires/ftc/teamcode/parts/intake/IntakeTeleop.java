package org.firstinspires.ftc.teamcode.parts.intake;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeTeleopSettings;

import om.self.ezftc.core.part.LoopedPartImpl;

public class IntakeTeleop extends LoopedPartImpl<Intake, IntakeTeleopSettings, ObjectUtils.Null> {
    private IntakeTeleopSettings settings;
    int myDelay = 1000;
    public IntakeTeleop(Intake parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeTeleopSettings.makeDefault(parent.parent));
    }

    public IntakeTeleop(Intake parent, IntakeTeleopSettings settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
    }

    public IntakeTeleopSettings getSettings() {
        return settings;
    }

    public void setSettings(IntakeTeleopSettings settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {

    }

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl(
                (int) settings.sweepSpeedSupplier.get(),
                (int) settings.sweepLiftSupplier.get(),
                (int) settings.robotLiftSupplier.get(),
                (int) settings.grabberSupplier.get(),
                (int) settings.launchReleaseSupplier.get(),
                (int) settings.startTagRanging.get(),
                (int) settings.startTagCentering.get()
        ), true);
    }

    @Override
    public void onRun() {
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }
}
