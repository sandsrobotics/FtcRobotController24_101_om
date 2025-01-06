package org.firstinspires.ftc.teamcode.parts.intake2.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware2 {
    public static final double slideHoldPower = 1;
    public static final double bucketHoldPower = 1;
    public final DcMotorEx bucketLiftMotor;
    public final DcMotorEx robotLiftMotor;
    public final Servo sliderServoLeft;
    public final Servo sliderServoRight;
    public final Servo tiltServoLeft;
    public final Servo tiltServoRight;
    public final Servo rotationServo;
    public final CRServo intakeWheelServoLeft;
    public final CRServo intakeWheelServoRight;
    public final DigitalChannel robotLiftZeroSwitch;
    public final DigitalChannel bucketLiftZeroSwitch;
    public final Servo dropperServo;
    public final Servo specimenServo;

    public IntakeHardware2(DcMotorEx bucketLiftMotor, DcMotorEx robotLiftMotor, Servo sliderServoLeft,
                           Servo sliderServoRight, Servo tiltServoLeft, Servo tiltServoRight, Servo rotationServo,
                           CRServo intakeWheelServoLeft, CRServo intakeWheelServoRight, DigitalChannel liftZeroSwitch,
                           DigitalChannel bucketLiftZeroSwitch, Servo dropperServo, Servo specimenServo) {
        this.bucketLiftMotor = bucketLiftMotor;
        this.robotLiftMotor = robotLiftMotor;
        this.sliderServoLeft = sliderServoLeft;
        this.sliderServoRight = sliderServoRight;
        this.tiltServoLeft = tiltServoLeft;
        this.tiltServoRight = tiltServoRight;
        this.rotationServo = rotationServo;
        this.intakeWheelServoLeft = intakeWheelServoLeft;
        this.intakeWheelServoRight = intakeWheelServoRight;
        this.robotLiftZeroSwitch = liftZeroSwitch;
        this.bucketLiftZeroSwitch = bucketLiftZeroSwitch;
        this.dropperServo = dropperServo;
        this.specimenServo = specimenServo;
    }

    public static IntakeHardware2 makeDefault(HardwareMap hardwareMap) {
        MotorSettings bucketLiftMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, bucketHoldPower);
        MotorSettings robotLift1MotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);

        ServoSettings sliderServoLeftSettings = new ServoSettings(ServoSettings.Number.THREE, Servo.Direction.FORWARD);
        ServoSettings sliderServoRightSettings = new ServoSettings(ServoSettings.Number.FOUR, Servo.Direction.REVERSE);
        ServoSettings tiltServoLeftSettings = new ServoSettings(ServoSettings.Number.FOUR_B, Servo.Direction.FORWARD);
        ServoSettings tiltServoRightSettings = new ServoSettings(ServoSettings.Number.FIVE_B, Servo.Direction.FORWARD);
        ServoSettings rotationServoSettings = new ServoSettings(ServoSettings.Number.TWO_B, Servo.Direction.FORWARD);
        ServoSettings dropperServoSettings = new ServoSettings(ServoSettings.Number.ZERO, Servo.Direction.FORWARD);
        ServoSettings specimenServoSettings = new ServoSettings(ServoSettings.Number.TWO, Servo.Direction.FORWARD);

        MotorSettings intakeWheelServoLeftSettings = new MotorSettings(ServoSettings.Number.THREE_B, DcMotorSimple.Direction.FORWARD);
        MotorSettings intakeWheelServoRightSettings = new MotorSettings(ServoSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE);
        DigitalChannel bucketLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital1");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);
        DigitalChannel robotLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital0");
        robotLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);

        return new IntakeHardware2(
                bucketLiftMotorSettings.makeExMotor(hardwareMap),
                robotLift1MotorSettings.makeExMotor(hardwareMap),
                sliderServoLeftSettings.makeServo(hardwareMap),
                sliderServoRightSettings.makeServo(hardwareMap),
                tiltServoRightSettings.makeServo(hardwareMap),
                tiltServoLeftSettings.makeServo(hardwareMap),
                rotationServoSettings.makeServo(hardwareMap),
                intakeWheelServoLeftSettings.makeCRServo(hardwareMap),
                intakeWheelServoRightSettings.makeCRServo(hardwareMap),
                bucketLiftZeroSwitch,
                robotLiftZeroSwitch,
                dropperServoSettings.makeServo(hardwareMap),
                specimenServoSettings.makeServo(hardwareMap)
        );
    }
}