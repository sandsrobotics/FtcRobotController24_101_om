package org.firstinspires.ftc.teamcode.parts.intake2.settings;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.Supplier;
import om.self.ezftc.core.Robot;
import om.self.supplier.suppliers.EdgeSupplier;

public class IntakeTeleopSettings2 {
    public final Supplier<Double> sweepSpeedSupplier;
    public final Supplier<Integer> sweepLiftSupplier;
    public final Supplier<Integer> sweepSlideSupplier;
    public final Supplier<Integer> bucketLiftSupplier;
    public final Supplier<Integer> robotLiftSupplier;
    public final Supplier<Integer> rotationServoSupplier;
    public final Supplier<Float> strafeSpeedSupplier;
    public final Supplier<Boolean> autoHomeSupplier;
    public final Supplier<Integer> specimenServoSupplier;
    public final Supplier<Boolean> robotEStopSupplier;
    public final Supplier<Boolean> rangeingSupplier;
    public final Supplier<Integer> autoSampleSupplier;

    public IntakeTeleopSettings2(Supplier<Double> sweepSpeedSupplier, Supplier<Integer> sweepLiftSupplier,
                                 Supplier<Integer> sweepSlideSupplier, Supplier<Integer> bucketLiftSupplier,
                                 Supplier<Integer> robotLiftSupplier,Supplier<Integer> rotationServoSupplier,
                                 Supplier<Float> strafeSpeedSupplier, Supplier<Boolean> autoHomeSupplier,
                                 Supplier<Integer> specimenServoSupplier, Supplier<Boolean> robotEStopSupplier,
                                 Supplier<Boolean> rangeingSupplier, Supplier<Integer> autoSampleSupplier) {
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        this.sweepLiftSupplier = sweepLiftSupplier;
        this.sweepSlideSupplier = sweepSlideSupplier;
        this.bucketLiftSupplier = bucketLiftSupplier;
        this.robotLiftSupplier = robotLiftSupplier;
        this.rotationServoSupplier = rotationServoSupplier;
        this.strafeSpeedSupplier = strafeSpeedSupplier;
        this.autoHomeSupplier = autoHomeSupplier;
        this.specimenServoSupplier = specimenServoSupplier;
        this.robotEStopSupplier = robotEStopSupplier;
        this.rangeingSupplier = rangeingSupplier;
        this.autoSampleSupplier = autoSampleSupplier;
    }

    public static IntakeTeleopSettings2 makeDefault(Robot robot) {
        Gamepad gamepad = robot.opMode.gamepad1;
        Gamepad gamepad2 = robot.opMode.gamepad2;

        return new IntakeTeleopSettings2(
                () -> gamepad.right_bumper ? -1 : gamepad.left_bumper ? 1 : 0.0,  // sweepSpeedSupplier
                () -> gamepad.y ? -1 : gamepad.a ? 1 : 0, // sweepLiftSupplier
                () -> gamepad.x ? -1 : gamepad.b ? 1 : 0, // horizontal SlideSupplier -1 = in
                () -> gamepad2.y ? 1 : gamepad2.a ? -1 : 0, // bucketLiftSupplier
                () -> gamepad2.left_bumper ? -1 : gamepad2.right_bumper ? 1 : 0, // robotLiftSupplier
                () -> gamepad2.x ? -1 : gamepad2.b ? 1 : 0, // rotationServoSupplier
                () -> gamepad2.left_stick_x, // strafeSpeedSupplier
                new EdgeSupplier(() -> gamepad.back).getRisingEdgeSupplier(), // autoHomeSupplier
                () -> gamepad2.dpad_left ? 1 : gamepad2.dpad_up ? 2 : gamepad2.dpad_right ? -1 : 0, // specimenServoSupplier
                () -> gamepad.right_stick_button || gamepad2.right_stick_button, // robotEStopSupplier
                () -> gamepad2.dpad_down, // range on off
                () -> gamepad.dpad_up ? 1 : gamepad.dpad_down ? -1 : 0 //
        );
    }
}
