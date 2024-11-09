package org.firstinspires.ftc.teamcode.depricated.lifter.settings;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

import om.self.ezftc.core.Robot;
import om.self.supplier.suppliers.EdgeSupplier;

public class LifterTeleopSettings {
    public final Supplier<Float> heightSpeedSupplier;
    public final Supplier<Float> turnSpeedSupplier;
    public final double turnSpeedMultiplier;
    public final Supplier<Boolean> grabberCloseSupplier;
    public final Supplier<Boolean> autoGrabSupplier;
    public final Supplier<Boolean> autoDockSupplier;
    public final Supplier<Boolean> autoDropSupplier;

    public final Supplier<Integer> preDropSupplier;
    public final Supplier<Integer> coneChangeSupplier; //change the height level of the cone

    public final Supplier<Boolean> autoHomeSupplier;
    public final Supplier<Boolean> forceCloseSupplier;

    public final Supplier<Boolean> forceDownSupplier;

    public LifterTeleopSettings(Supplier<Float> heightSpeedSupplier, Supplier<Float> turnSpeedSupplier, double turnSpeedMultiplier, Supplier<Boolean> grabberCloseSupplier, Supplier<Boolean> autoGrabSupplier, Supplier<Boolean> autoDockSupplier, Supplier<Boolean> autoDropSupplier, Supplier<Integer> preDropSupplier, Supplier<Integer> coneChangeSupplier, Supplier<Boolean> autoHomeSupplier, Supplier<Boolean> forceCloseSupplier, Supplier<Boolean> forceDownSupplier) {
        this.heightSpeedSupplier = heightSpeedSupplier;
        this.turnSpeedSupplier = turnSpeedSupplier;
        this.turnSpeedMultiplier = turnSpeedMultiplier;
        this.grabberCloseSupplier = grabberCloseSupplier;
        this.autoGrabSupplier = autoGrabSupplier;
        this.autoDockSupplier = autoDockSupplier;
        this.autoDropSupplier = autoDropSupplier;
        this.preDropSupplier = preDropSupplier;
        this.coneChangeSupplier = coneChangeSupplier;
        this.autoHomeSupplier = autoHomeSupplier;
        this.forceCloseSupplier = forceCloseSupplier;
        this.forceDownSupplier = forceDownSupplier;
    }

    public static LifterTeleopSettings makeDefault(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad2;

        EdgeSupplier downSupplier = new EdgeSupplier();
        downSupplier.setBase(() -> gamepad.left_bumper);

        EdgeSupplier upSupplier = new EdgeSupplier();
        upSupplier.setBase(() -> gamepad.right_bumper);

        EdgeSupplier autoDock = new EdgeSupplier();
        autoDock.setBase(() -> gamepad.x);

        EdgeSupplier autoDrop = new EdgeSupplier();
        autoDrop.setBase(() -> gamepad.b);
        EdgeSupplier autoGrab = new EdgeSupplier();
        autoGrab.setBase(() -> gamepad.y);

//        LatchedSupplier close = new LatchedSupplier(true);
//        close.setBase(() -> gamepad.a);

        return new LifterTeleopSettings(
                () -> gamepad.right_trigger - gamepad.left_trigger,
                () -> gamepad.left_stick_y,
                0.025,
                () -> !(robot.opMode.gamepad1.a),
                autoGrab::isRisingEdge,
                autoDock::isRisingEdge,
                autoDrop::isRisingEdge,
                () -> gamepad.dpad_right ? 0 : gamepad.dpad_down ? 1 : gamepad.dpad_left ? 2 : gamepad.dpad_up ? 3 : -1,
                () -> downSupplier.isRisingEdge() ? -1 : upSupplier.isRisingEdge() ? 1 : 0,
                new EdgeSupplier(() -> robot.opMode.gamepad1.x).getRisingEdgeSupplier(),
                () -> robot.opMode.gamepad1.y,
                () -> robot.opMode.gamepad1.dpad_down
        );
    }
}