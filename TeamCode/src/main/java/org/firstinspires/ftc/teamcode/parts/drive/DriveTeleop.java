package org.firstinspires.ftc.teamcode.parts.drive;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;

import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;
import om.self.ezftc.utils.VectorMath;

public class DriveTeleop extends LoopedPartImpl<Drive, DriveTeleopSettings, ObjectUtils.Null> {
    Vector3 invertVector = new Vector3(-1,-1,1);

    public DriveTeleop(Drive parent) {
        super(parent, "drive teleop");
        setSettings(DriveTeleopSettings.makeDefault(parent.parent));
    }

    public DriveTeleop(Drive parent, DriveTeleopSettings settings) {
        super(parent, "drive teleop");
        setSettings(settings);
    }

    @Override
    public void onBeanLoad() {

    }

    @Override
    public void onInit() {

    }

    @Override
    public void onRun() {
        parent.parent.opMode.telemetry.addData("Drive Speed", getSettings().slowModeSupplier.get() ? "slow" : getSettings().midModeSupplier.get() ? "mid" : "fast");
    }

    @Override
    public void onStart() {
        parent.setBaseController(() -> new DriveControl(
                getSettings().slowModeSupplier.get() ? VectorMath.multiply(getSettings().powerSupplier.get(), getSettings().slowModeSpeed) :
                getSettings().midModeSupplier.get() ? VectorMath.multiply(getSettings().powerSupplier.get(), getSettings().midModeSpeed) :
                getSettings().midModeSupplier.get() ? VectorMath.multiply(getSettings().powerSupplier.get(), invertVector) : getSettings().powerSupplier.get(), // fix this later by taking it out cause invert bad but im bored

                getSettings().stopSupplier.get()
        ), true);
    }

    @Override
    public void onStop() {
    }
}
