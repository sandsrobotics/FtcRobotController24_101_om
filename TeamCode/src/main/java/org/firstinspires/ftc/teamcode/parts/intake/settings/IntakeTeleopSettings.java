package org.firstinspires.ftc.teamcode.parts.intake.settings;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

import om.self.ezftc.core.Robot;
import om.self.supplier.suppliers.EdgeSupplier;

public class IntakeTeleopSettings {
    public final Supplier<Integer> sweepSpeedSupplier;
    public final Supplier<Integer> sweepLiftSupplier;
    public final Supplier<Integer> robotLiftSupplier;
    public final Supplier<Integer> grabberSupplier;
    public final Supplier<Integer> pixChangeSupplier;
    public final Supplier<Boolean> autoDropSupplier;
    public final Supplier<Boolean> autoDockSupplier;
    public final Supplier<Integer> launchReleaseSupplier;
    public final Supplier<Boolean> autoHomeSupplier;
    public final Supplier<Boolean> autoArmSupplier;
    public final Supplier<Boolean> autoStoreSupplier;
    public final Supplier<Integer> startTagRanging;
    public final Supplier<Integer> startTagCentering;
    public final Supplier<Boolean> releaseCenter;
    public final Supplier<Boolean> releaseRange;
    public final Supplier<Boolean> abortSupplier;

    public IntakeTeleopSettings(Supplier<Integer> sweepSpeedSupplier, Supplier<Integer> sweepLiftSupplier,
                                Supplier<Integer> robotLiftSupplier, Supplier<Integer> grabberSupplier,
                                Supplier<Integer> pixChangeSupplier, Supplier<Boolean> autoDropSupplier,
                                Supplier<Boolean> autoDockSupplier, Supplier<Integer> launchReleaseSupplier,
                                Supplier<Boolean> autoHomeSupplier, Supplier<Boolean> autoArmSupplier,
                                Supplier<Boolean> autoStoreSupplier, Supplier<Integer> startTagRanging,
                                Supplier<Integer> startTagCentering, Supplier<Boolean> releaseCenter,
                                Supplier<Boolean> releaseRange, Supplier<Boolean> abortSupplier){
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        this.sweepLiftSupplier = sweepLiftSupplier;
        this.robotLiftSupplier = robotLiftSupplier;
        this.grabberSupplier = grabberSupplier;
        this.pixChangeSupplier = pixChangeSupplier;
        this.autoDropSupplier = autoDropSupplier;
        this.autoDockSupplier = autoDockSupplier;
        this.launchReleaseSupplier = launchReleaseSupplier;
        this.autoHomeSupplier = autoHomeSupplier;
        this.autoArmSupplier = autoArmSupplier;
        this.autoStoreSupplier = autoStoreSupplier;
        this.startTagRanging = startTagRanging;
        this.startTagCentering = startTagCentering;
        this.releaseCenter = releaseCenter;
        this.releaseRange = releaseRange;
        this.abortSupplier = abortSupplier;
    }

    public static IntakeTeleopSettings makeDefault(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad1;
        Gamepad gamepad2 = robot.opMode.gamepad2;

        EdgeSupplier downSupplier = new EdgeSupplier();
        downSupplier.setBase(() -> gamepad2.left_bumper);

        EdgeSupplier upSupplier = new EdgeSupplier();
        upSupplier.setBase(() -> gamepad2.right_bumper);

        EdgeSupplier autoDrop = new EdgeSupplier();
        autoDrop.setBase(() -> gamepad2.dpad_up);

        EdgeSupplier autoDock = new EdgeSupplier();
        autoDock.setBase(()-> gamepad2.dpad_down);

        EdgeSupplier autoArm = new EdgeSupplier();
        autoArm.setBase(()->gamepad.dpad_left);

        EdgeSupplier autoStore = new EdgeSupplier();
        autoStore.setBase(()->gamepad2.dpad_left);

        EdgeSupplier releaseCenter = new EdgeSupplier();
        releaseCenter.setBase(()->gamepad.y);

        EdgeSupplier releaseRange = new EdgeSupplier();
        releaseRange.setBase(()->gamepad.b);


        return new IntakeTeleopSettings(
            ()-> gamepad.right_bumper ? -1 : gamepad.left_bumper ? 1 : 0,
            () -> gamepad2.right_stick_button ? 1 : gamepad2.left_stick_button ? 2 : 0, //sweep lift supplier
            () -> gamepad.dpad_down ? -1 : gamepad.dpad_up ? 1 : 0,
            () -> gamepad2.y ? 1 : gamepad2.b ? 2 : gamepad2.a ? 3 : 0,
            () -> downSupplier.isRisingEdge() ? -1 : upSupplier.isRisingEdge() ? 1 : 0,
//                upSupplier::isRisingEdge,
            autoDrop::isRisingEdge,
            autoDock::isRisingEdge,
            () -> gamepad2.x && gamepad2.dpad_right ? 1 : 0,
            new EdgeSupplier(()-> robot.opMode.gamepad1.back).getRisingEdgeSupplier(),
                autoArm::isRisingEdge,
                autoStore::isRisingEdge,
            ()-> gamepad.b ? 1 : 0,
            ()-> gamepad.y ? 1 : 0,
                releaseCenter::isFallingEdge,
                releaseRange::isFallingEdge,
                new EdgeSupplier(()->robot.opMode.gamepad1.a).getRisingEdgeSupplier()
        );
    }
}