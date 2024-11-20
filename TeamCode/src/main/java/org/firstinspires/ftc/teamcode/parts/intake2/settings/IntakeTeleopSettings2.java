package org.firstinspires.ftc.teamcode.parts.intake2.settings;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.Supplier;
import om.self.ezftc.core.Robot;

public class IntakeTeleopSettings2 {
    public final Supplier<Integer> sweepSpeedSupplier;
    public final Supplier<Integer> sweepLiftSupplier;
    public final Supplier<Integer> sweepSlideSupplier;
    public final Supplier<Integer> bucketLiftSupplier;

    public IntakeTeleopSettings2(Supplier<Integer> sweepSpeedSupplier, Supplier<Integer> sweepLiftSupplier,
                                 Supplier<Integer> sweepSlideSupplier, Supplier<Integer> bucketLiftSupplier){
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        this.sweepLiftSupplier = sweepLiftSupplier;
        this.sweepSlideSupplier = sweepSlideSupplier;
        this.bucketLiftSupplier = bucketLiftSupplier;
    }

    public static IntakeTeleopSettings2 makeDefault(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad1;
        Gamepad gamepad2 = robot.opMode.gamepad2;

        return new IntakeTeleopSettings2(
            ()-> gamepad.right_bumper ? -1 : gamepad.left_bumper ? 1 : 0,  // sweepSpeedSupplier
            () -> gamepad.a ? 1 : gamepad.y ? 2 : 0, //sweep lift supplier
            () -> gamepad.x ? 0 : gamepad.b ? 1 : -1, //sweep slide supplier
//            () -> gamepad.dpad_left ? -1 : gamepad.dpad_right ? 1 : 0, // sweep slide Supplier
            () -> gamepad.dpad_down ? -1 : gamepad.dpad_up ? 1 : 0 // bucketLift Supplier
        );
    }
}