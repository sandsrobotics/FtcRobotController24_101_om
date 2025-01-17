package org.firstinspires.ftc.teamcode.parts.intake2.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware2 {
    public static final double slideHoldPower = 1;
    public static final double bucketHoldPower = 1;
    public final DcMotorEx bucketLiftMotor;
    public final DcMotorEx robotLiftMotor;
    public final Servo sliderServoLeft;
    public final Servo sliderServoRight;
    public final ServoSSR tiltServo;
    public final Servo rotationServo;
    public final CRServo intakeWheelServoLeft;
    public final CRServo intakeWheelServoRight;
    public final DigitalChannel robotLiftZeroSwitch;
    public final DigitalChannel bucketLiftZeroSwitch;
    public final ServoSSR dropperServo;
    public final ServoSSR specimenServo;
    //public final Rev2mDistanceSensor specDistance;

    public IntakeHardware2(DcMotorEx bucketLiftMotor, DcMotorEx robotLiftMotor, Servo sliderServoLeft,
                           Servo sliderServoRight, ServoSSR tiltServo, Servo rotationServo,
                           CRServo intakeWheelServoLeft, CRServo intakeWheelServoRight, DigitalChannel liftZeroSwitch,
                           DigitalChannel bucketLiftZeroSwitch, ServoSSR dropperServo, ServoSSR specimenServo){
                           //Rev2mDistanceSensor specDistance) {
        this.bucketLiftMotor = bucketLiftMotor;
        this.robotLiftMotor = robotLiftMotor;
        this.sliderServoLeft = sliderServoLeft;
        this.sliderServoRight = sliderServoRight;
        this.tiltServo = tiltServo;
        this.rotationServo = rotationServo;
        this.intakeWheelServoLeft = intakeWheelServoLeft;
        this.intakeWheelServoRight = intakeWheelServoRight;
        this.robotLiftZeroSwitch = liftZeroSwitch;
        this.bucketLiftZeroSwitch = bucketLiftZeroSwitch;
        this.dropperServo = dropperServo;
        this.specimenServo = specimenServo;
        //this.specDistance = specDistance;
    }

    public static IntakeHardware2 makeDefault(HardwareMap hardwareMap) {
        MotorSettings bucketLiftMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, bucketHoldPower);
        MotorSettings robotLift1MotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
        ServoSettings sliderServoLeftSettings = new ServoSettings(ServoSettings.Number.THREE, Servo.Direction.FORWARD);
        ServoSettings sliderServoRightSettings = new ServoSettings(ServoSettings.Number.FOUR, Servo.Direction.REVERSE);
        ServoSettings tiltServoSettings = new ServoSettings(ServoSettings.Number.FIVE_B, Servo.Direction.FORWARD);
        ServoSettings rotationServoSettings = new ServoSettings(ServoSettings.Number.TWO_B, Servo.Direction.FORWARD);
        ServoSettings dropperServoSettings = new ServoSettings(ServoSettings.Number.ZERO, Servo.Direction.FORWARD);
        ServoSettings specimenServoSettings = new ServoSettings(ServoSettings.Number.TWO, Servo.Direction.FORWARD);

        MotorSettings intakeWheelServoLeftSettings = new MotorSettings(ServoSettings.Number.THREE_B, DcMotorSimple.Direction.FORWARD);
        MotorSettings intakeWheelServoRightSettings = new MotorSettings(ServoSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE);
        DigitalChannel bucketLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital1");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);
        DigitalChannel robotLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital0");
        robotLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);
        //Rev2mDistanceSensor specDistance = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_distance");

        return new IntakeHardware2(
                bucketLiftMotorSettings.makeExMotor(hardwareMap),
                robotLift1MotorSettings.makeExMotor(hardwareMap),
                sliderServoLeftSettings.makeServo(hardwareMap),
                sliderServoRightSettings.makeServo(hardwareMap),
                tiltServoSettings.makeServoSSR(hardwareMap),
                rotationServoSettings.makeServo(hardwareMap),
                intakeWheelServoLeftSettings.makeCRServo(hardwareMap),
                intakeWheelServoRightSettings.makeCRServo(hardwareMap),
                bucketLiftZeroSwitch,
                robotLiftZeroSwitch,
                dropperServoSettings.makeServoSSR(hardwareMap),
                specimenServoSettings.makeServoSSR(hardwareMap)
                //specDistance
        );
    }
}