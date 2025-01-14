package org.firstinspires.ftc.teamcode.parts.intake.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware {
    public static final double slideHoldPower = 1;
    public static final double bucketHoldPower = 1;
    public final DcMotorEx bucketLiftMotor;
    public final DcMotorEx horizSliderMotor;
//    public final Servo tiltServo;
//    public final CRServo intakeFlipperServo;
    public final ServoSSR spinner;
    public final ServoSSR flipper;
    public final ServoSSR chute;
    public final ServoSSR pinch;
    public final NormalizedColorSensor colorSensor;

    public final DigitalChannel bucketLiftZeroSwitch;
    public final DigitalChannel slideZeroSwitch;

    public IntakeHardware(DcMotorEx bucketLiftMotor, DcMotorEx slideMotor, DigitalChannel bucketLiftZeroSwitch, DigitalChannel slideZeroSwitch,
                          ServoSSR spinner, ServoSSR flipper, ServoSSR chute, ServoSSR pinch, NormalizedColorSensor colorSensor) {
        this.bucketLiftMotor = bucketLiftMotor;
        this.horizSliderMotor = slideMotor;
//        this.tiltServo = tiltServo;
//        this.intakeFlipperServo = intakeFlipperServo;
        this.bucketLiftZeroSwitch = bucketLiftZeroSwitch;
        this.slideZeroSwitch = slideZeroSwitch;
        this.spinner = spinner;
        this.flipper = flipper;
        this.chute = chute;
        this.pinch = pinch;
        this.colorSensor = colorSensor;
    }
//beans
    public static IntakeHardware makeDefault(HardwareMap hardwareMap)  {
        MotorSettings bucketLiftMotorSettings =new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, bucketHoldPower);
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
//        ServoSettings tiltServoSettings = new ServoSettings(ServoSettings.Number.FOUR, Servo.Direction.FORWARD);ServoSettings tiltServoRightSettings = new ServoSettings(ServoSettings.Number.FIVE, Servo.Direction.FORWARD);
//        MotorSettings intakeFlipperServoSettings = new MotorSettings(ServoSettings.Number.ZERO, DcMotorSimple.Direction.FORWARD);MotorSettings intakeWheelServoRightSettings = new MotorSettings(ServoSettings.Number.TWO, DcMotorSimple.Direction.REVERSE);
        DigitalChannel bucketLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital2");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);
        DigitalChannel slideZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital0");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);

        return new IntakeHardware(
                bucketLiftMotorSettings.makeExMotor(hardwareMap),
                slideMotorSettings.makeExMotor(hardwareMap),
//                tiltServoSettings.makeServo(hardwareMap),
//                intakeFlipperServoSettings.makeCRServo(hardwareMap),
                bucketLiftZeroSwitch,
                slideZeroSwitch,
                new ServoSSR(hardwareMap.get(Servo.class,"servo0")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo2")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo4")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo1")),
                hardwareMap.get(NormalizedColorSensor.class, "color")
        );
    }
}