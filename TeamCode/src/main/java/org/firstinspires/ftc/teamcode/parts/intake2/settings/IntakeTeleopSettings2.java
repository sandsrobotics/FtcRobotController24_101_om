package org.firstinspires.ftc.teamcode.parts.intake2.settings;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.Supplier;
import om.self.ezftc.core.Robot;

public class IntakeTeleopSettings2 {
    public final Supplier<Integer> sweepSpeedSupplier;
    public final Supplier<Integer> sweepLiftSupplier;
    public final Supplier<Integer> sweepSlideSupplier;
    public final Supplier<Integer> bucketLiftSupplier;
    public final Supplier<Integer> robotLiftSupplier;
    public final Supplier<Integer> robotLift0Supplier;
    public final Supplier<Integer> robotLifthangSupplier;
    public final Supplier<Integer> rotationServoSupplier;

    public IntakeTeleopSettings2(Supplier<Integer> sweepSpeedSupplier, Supplier<Integer> sweepLiftSupplier,
                                 Supplier<Integer> sweepSlideSupplier, Supplier<Integer> bucketLiftSupplier,
                                 Supplier<Integer> robotLiftSupplier, Supplier<Integer> robotLift0Supplier,
                                 Supplier<Integer> robotLifthangSupplier, Supplier<Integer> rotationServoSupplier) {
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        this.sweepLiftSupplier = sweepLiftSupplier;
        this.sweepSlideSupplier = sweepSlideSupplier;
        this.bucketLiftSupplier = bucketLiftSupplier;
        this.robotLiftSupplier = robotLiftSupplier;
        this.robotLift0Supplier = robotLift0Supplier;
        this.robotLifthangSupplier = robotLifthangSupplier;
        this.rotationServoSupplier = rotationServoSupplier;
    }

    public static IntakeTeleopSettings2 makeDefault(Robot robot) {
        Gamepad gamepad = robot.opMode.gamepad1;
        Gamepad gamepad2 = robot.opMode.gamepad2;

        return new IntakeTeleopSettings2(
                () -> gamepad.right_bumper ? -1 : gamepad.left_bumper ? 1 : 0,  // sweepSpeedSupplier
                () -> gamepad.a ? 1 : gamepad.y ? 2 : 0, // sweepLiftSupplier
                () -> gamepad.x ? 0 : gamepad.b ? 1 : -1, // sweepSlideSupplier
                () -> gamepad.dpad_down ? -1 : gamepad.dpad_up ? 1 : 0, // bucketLiftSupplier
                () -> gamepad2.dpad_down ? -1 : gamepad2.dpad_up ? 1 : 0, // robotLiftSupplier
                () -> gamepad2.a ? 1 : 0, // robotLift0Supplier
                () -> gamepad2.x ? -1 : 0, // robotLifthangSupplier
                () -> gamepad.dpad_left ? -1 : gamepad.dpad_right ? 1 : 0 // rotationServoSupplier
        );
    }
}
