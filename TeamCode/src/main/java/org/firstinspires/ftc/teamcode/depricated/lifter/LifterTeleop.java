package org.firstinspires.ftc.teamcode.depricated.lifter;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.depricated.lifter.settings.LifterTeleopSettings;

import java.util.function.Supplier;

import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.supplier.suppliers.EdgeSupplier;

public class LifterTeleop extends LoopedPartImpl<Lifter, LifterTeleopSettings, ObjectUtils.Null> {
    private LifterTeleopSettings settings;

    private EdgeSupplier preDropEdge = new EdgeSupplier();
//    private Supplier<Boolean> coneInRangeEdge = new EdgeSupplier(parent::isConeInRangeTeleOp).getRisingEdgeSupplier();

    Supplier<Boolean> magic = new EdgeSupplier(() -> parent.parent.opMode.gamepad2.a).getRisingEdgeSupplier();

    public LifterTeleop(Lifter parent) {
        super(parent, "lifter teleop");
        setSettings(LifterTeleopSettings.makeDefault(parent.parent));
    }

    public LifterTeleop(Lifter parent, LifterTeleopSettings settings) {
        super(parent, "lifter teleop");
        setSettings(settings);
    }

    public LifterTeleopSettings getSettings() {
        return settings;
    }

    public void setSettings(LifterTeleopSettings settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
        preDropEdge.setBase(() -> settings.preDropSupplier.get() > -1);
    }

    @Override
    public void onStart() {
        parent.setBaseController(() -> new LifterControl(
                (double) settings.heightSpeedSupplier.get(),
                (double) settings.turnSpeedSupplier.get() * settings.turnSpeedMultiplier,
                settings.grabberCloseSupplier.get(),
                settings.forceDownSupplier.get()
        ), true);
    }

    @Override
    public void onRun() {
        int cone = settings.coneChangeSupplier.get();

        if(cone != 0){
            parent.setCone(parent.getCone() + cone);
            parent.startAutoDock();
        }

        if(preDropEdge.isRisingEdge()){ //&& parent.isGrabberClosed()){
            parent.setPole(settings.preDropSupplier.get());
            parent.startAutoPreDrop();
        }
        else if(settings.autoGrabSupplier.get())//  || coneInRangeEdge.get())
            parent.startAutoGrab();
        else if(settings.autoDockSupplier.get())
            parent.startAutoDock();
        else if(settings.autoDropSupplier.get())// && parent.isGrabberClosed())
            parent.startAutoDrop2();

        if(magic.get()){
            parent.setLiftPosition(parent.getLiftPosition() - 190);
        }

        if(parent.parent.opMode.gamepad2.start || parent.parent.opMode.gamepad1.start){
            parent.emergencyStop();
        }

        if(settings.autoHomeSupplier.get())
            parent.startAutoHome();

        if(settings.forceCloseSupplier.get()){
            LifterControl.flipOpen = 0;
            parent.setGrabberClosed();
        }

        parent.parent.opMode.telemetry.addData("cone", parent.getCone() + 1);
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }
}