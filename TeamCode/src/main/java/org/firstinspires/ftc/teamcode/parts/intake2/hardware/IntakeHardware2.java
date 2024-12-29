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
    public final DigitalChannel liftZeroSwitch;

    public IntakeHardware2(DcMotorEx bucketLiftMotor, DcMotorEx robotLiftMotor, Servo sliderServoLeft,
                           Servo sliderServoRight, Servo tiltServoLeft, Servo tiltServoRight, Servo rotationServo,
                           CRServo intakeWheelServoLeft, CRServo intakeWheelServoRight, DigitalChannel liftZeroSwitch) {
        this.bucketLiftMotor = bucketLiftMotor;
        this.robotLiftMotor = robotLiftMotor;
        this.sliderServoLeft = sliderServoLeft;
        this.sliderServoRight = sliderServoRight;
        this.tiltServoLeft = tiltServoLeft;
        this.tiltServoRight = tiltServoRight;
        this.rotationServo = rotationServo;
        this.intakeWheelServoLeft = intakeWheelServoLeft;
        this.intakeWheelServoRight = intakeWheelServoRight;
        this.liftZeroSwitch = liftZeroSwitch;
    }

    public static IntakeHardware2 makeDefault(HardwareMap hardwareMap) {
        MotorSettings bucketLiftMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, bucketHoldPower);
        MotorSettings robotLift1MotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
        ServoSettings sliderServoLeftSettings = new ServoSettings(ServoSettings.Number.THREE, Servo.Direction.FORWARD);
        ServoSettings sliderServoRightSettings = new ServoSettings(ServoSettings.Number.FOUR, Servo.Direction.REVERSE);
        ServoSettings tiltServoLeftSettings = new ServoSettings(ServoSettings.Number.FOUR_B, Servo.Direction.FORWARD);
        ServoSettings tiltServoRightSettings = new ServoSettings(ServoSettings.Number.FIVE_B, Servo.Direction.FORWARD);
        ServoSettings rotationServoSettings = new ServoSettings(ServoSettings.Number.TWO_B, Servo.Direction.FORWARD);
        MotorSettings intakeWheelServoLeftSettings = new MotorSettings(ServoSettings.Number.THREE_B, DcMotorSimple.Direction.FORWARD);
        MotorSettings intakeWheelServoRightSettings = new MotorSettings(ServoSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE);
        DigitalChannel liftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital0");
        liftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);

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
                liftZeroSwitch
        );
    }
}