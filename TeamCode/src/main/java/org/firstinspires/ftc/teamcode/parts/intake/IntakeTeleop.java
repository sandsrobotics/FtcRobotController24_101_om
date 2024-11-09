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
        parent.dropCounter = 0;
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
        int pix = settings.pixChangeSupplier.get();

        if(pix != 0){
            parent.setPix(parent.getPix() + pix);
        }

        if(settings.autoDropSupplier.get())
            parent.startAutoDrop();
        else if(settings.autoDockSupplier.get())
            parent.startAutoDock();
        else if(settings.autoHomeSupplier.get())
            parent.startAutoHome();
        else if(settings.autoArmSupplier.get())
            parent.startAutoArm();
//        else if(settings.autoStoreSupplier.get())
//            parent.startAutoStore();
        else if(settings.releaseRange.get()) {
            parent.dropCounter++;
            parent.rangingHeld = true;
            parent.lastBackDist = 322;
            if(parent.dropCounter == 2) {
                parent.startAutoDock();
                parent.dropCounter = 0;
            }
        }
        else if(settings.releaseCenter.get())
            parent.startRunCenter();
        else if(settings.abortSupplier.get())
            parent.abortRange = false;


        parent.parent.opMode.telemetry.addData("pix", parent.getPix() + 1);
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }
}
