package org.firstinspires.ftc.teamcode.parts.intake2.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.lib.ServoSSR;
import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware2 {
    public static final double slideHoldPower = 1;
    public static final double bucketHoldPower = 1;
    public final DcMotorEx bucketLiftMotor;
    public final DcMotorEx robotLiftMotor;
    public final ServoSSR sliderServoLeft;
    public final ServoSSR sliderServoRight;
    public final ServoSSR tiltServo;
    public final ServoSSR rotationServo;
    public final ServoSSR intakeWheelServoLeft;
    public final ServoSSR intakeWheelServoRight;
    public final DigitalChannel robotLiftZeroSwitch;
    public final DigitalChannel bucketLiftZeroSwitch;
    public final ServoSSR dropperServo;
    public final ServoSSR specimenServo;
    public final ServoSSR parkServo;
    public final ServoSSR backLight;
    public final Rev2mDistanceSensor rangeSensor;
    //public final RevColorSensorV3 specColorSensor;
    public final NormalizedColorSensor colorSensor;

    public IntakeHardware2(DcMotorEx bucketLiftMotor, DcMotorEx robotLiftMotor, ServoSSR sliderServoLeft,
                           ServoSSR sliderServoRight, ServoSSR tiltServo, ServoSSR rotationServo,
                           ServoSSR intakeWheelServoLeft, ServoSSR intakeWheelServoRight, DigitalChannel liftZeroSwitch,
                           DigitalChannel bucketLiftZeroSwitch, ServoSSR dropperServo, ServoSSR specimenServo,
                           ServoSSR parkServo, ServoSSR backLight, Rev2mDistanceSensor rangeSensor, NormalizedColorSensor colorSensor){
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
        this.parkServo = parkServo;
        this.backLight = backLight;
        this.rangeSensor = rangeSensor;
        this.colorSensor = colorSensor;
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
        ServoSettings parkServoSettings = new ServoSettings(ServoSettings.Number.ONE, Servo.Direction.FORWARD);
        ServoSettings backLight = new ServoSettings(ServoSettings.Number.FIVE, Servo.Direction.FORWARD);

        ServoSettings intakeWheelServoLeftSettings = new ServoSettings(ServoSettings.Number.THREE_B, Servo.Direction.FORWARD);
        ServoSettings intakeWheelServoRightSettings = new ServoSettings(ServoSettings.Number.ONE_B, Servo.Direction.REVERSE);


        Rev2mDistanceSensor rangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "range_sensor");
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        DigitalChannel bucketLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital1");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);
        DigitalChannel robotLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital0");
        robotLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);
        //Rev2mDistanceSensor specDistance = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_distance");

        return new IntakeHardware2(
                bucketLiftMotorSettings.makeExMotor(hardwareMap),
                robotLift1MotorSettings.makeExMotor(hardwareMap),
                sliderServoLeftSettings.makeServoSSR(hardwareMap),
                sliderServoRightSettings.makeServoSSR(hardwareMap),
                tiltServoSettings.makeServoSSR(hardwareMap),
                rotationServoSettings.makeServoSSR(hardwareMap),
                intakeWheelServoLeftSettings.makeServoSSR(hardwareMap),
                intakeWheelServoRightSettings.makeServoSSR(hardwareMap),
                bucketLiftZeroSwitch,
                robotLiftZeroSwitch,
                dropperServoSettings.makeServoSSR(hardwareMap),
                specimenServoSettings.makeServoSSR(hardwareMap),
                parkServoSettings.makeServoSSR(hardwareMap),
                backLight.makeServoSSR(hardwareMap),
                rangeSensor,
                colorSensor
                //specDistance
        );
    }
}