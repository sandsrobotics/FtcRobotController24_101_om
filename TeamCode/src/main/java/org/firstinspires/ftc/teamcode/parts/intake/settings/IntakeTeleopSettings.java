package org.firstinspires.ftc.teamcode.parts.intake.settings;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.Supplier;
import om.self.ezftc.core.Robot;

public class IntakeTeleopSettings {
    public final Supplier<Integer> sweepSpeedSupplier;
    public final Supplier<Integer> sweepLiftSupplier;
    public final Supplier<Integer> sweepSlideSupplier;
    public final Supplier<Integer> bucketLiftSupplier;
    public final Supplier<Integer> specimanClawSupplier;
    public final Supplier<Integer> v_SlideSupplier;
    public final Supplier<Integer> intakeAngleSupplier;
    public IntakeTeleopSettings(Supplier<Integer> sweepSpeedSupplier, Supplier<Integer> sweepLiftSupplier,
                                Supplier<Integer> sweepSlideSupplier, Supplier<Integer> bucketLiftSupplier,
                                Supplier<Integer> specimanClawSupplier, Supplier<Integer> v_SlideSupplier,
                                Supplier<Integer> intakeAngleSupplier){
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        this.sweepLiftSupplier = sweepLiftSupplier;
        this.sweepSlideSupplier = sweepSlideSupplier;
        this.bucketLiftSupplier = bucketLiftSupplier;
        this.specimanClawSupplier = specimanClawSupplier;
        this.v_SlideSupplier = v_SlideSupplier;
        this.intakeAngleSupplier = intakeAngleSupplier;
    }

    public static IntakeTeleopSettings makeDefault(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad1;
        Gamepad gamepad2 = robot.opMode.gamepad2;

        return new IntakeTeleopSettings(
            () -> gamepad.right_bumper ? -1 : gamepad.left_bumper ? 1 : 0,  // sweepSpeedSupplier
            () -> gamepad.y ? 1 : gamepad.a ? 2 : 0, //sweep lift supplier
            () -> gamepad2.x ? 0 : gamepad2.b ? 1 : -1, //sweep slide supplier
            () -> gamepad.dpad_down ? -1 : gamepad.dpad_up ? 1 : 0, // bucketLift Supplier
            () -> gamepad2.dpad_up ? -1 : gamepad2.dpad_down ? 1 : 0,//specimanclawSupplier
            () -> gamepad2.dpad_down ? -1 : gamepad2.dpad_up ? 1 : gamepad2.dpad_right ? -2 : gamepad2.dpad_left ? 2 : 0,// v_slide supplier
        () -> gamepad2.a ? -1 : gamepad2.y ? 1: 0
        );
    }
}