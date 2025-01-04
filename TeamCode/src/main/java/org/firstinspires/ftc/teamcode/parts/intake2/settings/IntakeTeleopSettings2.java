package org.firstinspires.ftc.teamcode.parts.intake2.settings;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.Supplier;
import om.self.ezftc.core.Robot;
import om.self.supplier.suppliers.EdgeSupplier;

public class IntakeTeleopSettings2 {
    public final Supplier<Integer> sweepSpeedSupplier;
    public final Supplier<Integer> sweepLiftSupplier;
    public final Supplier<Integer> sweepSlideSupplier;
    public final Supplier<Integer> bucketLiftSupplier;
    public final Supplier<Integer> robotLiftSupplier;
    public final Supplier<Integer> robotLift0Supplier;
    public final Supplier<Integer> robotLifthangSupplier;
    public final Supplier<Integer> rotationServoSupplier;
    public final Supplier<Float> strafeSpeedSupplier;
//    public final Supplier<Boolean> robotLiftPrepSupplier;
//    public final Supplier<Boolean> robotLiftNowSupplier;

    public IntakeTeleopSettings2(Supplier<Integer> sweepSpeedSupplier, Supplier<Integer> sweepLiftSupplier,
                                 Supplier<Integer> sweepSlideSupplier, Supplier<Integer> bucketLiftSupplier,
                                 Supplier<Integer> robotLiftSupplier, Supplier<Integer> robotLift0Supplier,
                                 Supplier<Integer> robotLifthangSupplier, Supplier<Integer> rotationServoSupplier,
                                 Supplier<Float> strafeSpeedSupplier) {
//                                 Supplier<Boolean> robotLiftPrepSupplier, Supplier<Boolean> robotLiftNowSupplier) {
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        this.sweepLiftSupplier = sweepLiftSupplier;
        this.sweepSlideSupplier = sweepSlideSupplier;
        this.bucketLiftSupplier = bucketLiftSupplier;
        this.robotLiftSupplier = robotLiftSupplier;
        this.robotLift0Supplier = robotLift0Supplier;
        this.robotLifthangSupplier = robotLifthangSupplier;
        this.rotationServoSupplier = rotationServoSupplier;
        this.strafeSpeedSupplier = strafeSpeedSupplier;
//        this.robotLiftPrepSupplier = robotLiftPrepSupplier;
//        this.robotLiftNowSupplier = robotLiftNowSupplier;
    }

    public static IntakeTeleopSettings2 makeDefault(Robot robot) {
        Gamepad gamepad = robot.opMode.gamepad1;
        Gamepad gamepad2 = robot.opMode.gamepad2;

//        EdgeSupplier liftPrep = new EdgeSupplier();
//        liftPrep.setBase(() -> gamepad2.dpad_up);
//
//        EdgeSupplier liftNow = new EdgeSupplier();
//        liftNow.setBase(() -> gamepad2.dpad_down);

        return new IntakeTeleopSettings2(
                () -> gamepad.right_bumper ? -1 : gamepad.left_bumper ? 1 : 0,  // sweepSpeedSupplier
                () -> gamepad.a ? -1 : gamepad.y ? 1 : 0, // sweepLiftSupplier
                () -> gamepad.b ? -1 : gamepad.x ? 1 : 0, // horizontal SlideSupplier -1 = in
                () -> gamepad2.a ? 1 : gamepad2.b ? -1 : 0, // bucketLiftSupplier
                () -> gamepad2.x ? 1 : 0, // robotLiftSupplier now triggers setRobotLiftPosition
                () -> 0, // robotLift0Supplier (disabled, always 0)
                () -> gamepad2.y ? 1 : 0, // dropperServo control
                () -> gamepad.dpad_left ? -1 : gamepad.dpad_right ? 1 : 0, // rotationServoSupplier
                () -> gamepad2.left_stick_x // strafe speed
        );

    }
}
