package org.firstinspires.ftc.teamcode.parts.intake.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware {
    public static final double slideHoldPower = 1;
    public static final double sweepHoldPower = 1;
    public static final double robotLiftHoldPower = 1;
    public static final double swingHoldPower = 1;

    public final DcMotorEx sliderMotor;
    public final DcMotorEx sweeperMotor;
    public final Servo sweepLiftServo;
    public final DcMotorEx robotLiftMotor;
    public final Servo grabberServo;
    public final DigitalChannel liftLowLimitSwitch;
    public final DigitalChannel liftHighLimitSwitch;
    public final DigitalChannel slideLowLimitSwitch;
    public final Servo swingServoLeft;
    public final Servo swingServoRight;
    public final Servo launchServoAngle;
    public final Servo launchServoRelease;
    public final RevColorSensorV3 botSensor;
    public final RevColorSensorV3 topSensor;
    public final Rev2mDistanceSensor backSensor;
    public final Rev2mDistanceSensor redSensor;
    public final Rev2mDistanceSensor blueSensor;
    public final DigitalChannel grabberLimitSwitch;

    public IntakeHardware(DcMotorEx sliderMotor, DcMotorEx sweeperMotor,
                          Servo sweepLiftServo, DcMotorEx robotLiftMotor,
                          Servo grabberServo, DigitalChannel liftLowLimitSwitch,
                          DigitalChannel liftHighLImitSwitch, DigitalChannel slideLowLimitSwitch,
                          Servo swingServoLeft,Servo swingServoRight, Servo launchServoAngle,
                          Servo launchServoRelease, RevColorSensorV3 botSensor, RevColorSensorV3 topSensor,
                          Rev2mDistanceSensor backSensor, DigitalChannel grabberLimitSwitch, Rev2mDistanceSensor redSensor,
                          Rev2mDistanceSensor blueSensor) {
        this.sweeperMotor = sweeperMotor;
        this.sliderMotor = sliderMotor;
        this.sweepLiftServo = sweepLiftServo;
        this.robotLiftMotor = robotLiftMotor;
        this.grabberServo = grabberServo;
        this.liftLowLimitSwitch = liftLowLimitSwitch;
        this.liftHighLimitSwitch = liftHighLImitSwitch;
        this.slideLowLimitSwitch = slideLowLimitSwitch;
        this.swingServoLeft = swingServoLeft;
        this.swingServoRight = swingServoRight;
        this.launchServoAngle = launchServoAngle;
        this.launchServoRelease = launchServoRelease;
        this.botSensor = botSensor;
        this.topSensor = topSensor;
        this.backSensor = backSensor;
        this.grabberLimitSwitch = grabberLimitSwitch;
        this.redSensor = redSensor;
        this.blueSensor = blueSensor;
    }
//beans
    public static IntakeHardware makeDefault(HardwareMap hardwareMap) {
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
        MotorSettings sweepMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, sweepHoldPower);
        ServoSettings sweepLiftServoSettings = new ServoSettings(ServoSettings.Number.ZERO_B, Servo.Direction.FORWARD);
        MotorSettings robotLiftMotorSettings = new MotorSettings(MotorSettings.Number.TWO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, robotLiftHoldPower);
        ServoSettings grabberServoSettings = new ServoSettings(ServoSettings.Number.ZERO, Servo.Direction.FORWARD);
        ServoSettings swingServoLeftSettings = new ServoSettings(ServoSettings.Number.ONE, Servo.Direction.FORWARD);
        ServoSettings swingServoRightSettings = new ServoSettings(ServoSettings.Number.TWO, Servo.Direction.REVERSE);
        ServoSettings launchServoAngleSettings = new ServoSettings(ServoSettings.Number.THREE_B, Servo.Direction.FORWARD);
        ServoSettings launchServoReleaseSettings = new ServoSettings(ServoSettings.Number.THREE, Servo.Direction.FORWARD);
        RevColorSensorV3 botSensor = hardwareMap.get(RevColorSensorV3.class, "botSensor");
        RevColorSensorV3 topSensor = hardwareMap.get(RevColorSensorV3.class, "topSensor");
        Rev2mDistanceSensor backSensor = hardwareMap.get(Rev2mDistanceSensor.class, "backSensor");
        DigitalChannel lowLiftLimit = hardwareMap.get(DigitalChannel.class, "digital0");
        DigitalChannel highLiftLimit = hardwareMap.get(DigitalChannel.class, "digital1");
        DigitalChannel lowSlideLimit = hardwareMap.get(DigitalChannel.class, "digital3");
        DigitalChannel grabberLimit = hardwareMap.get(DigitalChannel.class, "digital6");
        lowLiftLimit.setMode(DigitalChannel.Mode.INPUT);
        highLiftLimit.setMode(DigitalChannel.Mode.INPUT);
        lowSlideLimit.setMode(DigitalChannel.Mode.INPUT);
        grabberLimit.setMode(DigitalChannel.Mode.INPUT);
        Rev2mDistanceSensor redSensor = hardwareMap.get(Rev2mDistanceSensor.class, "redSensor");
        Rev2mDistanceSensor blueSensor = hardwareMap.get(Rev2mDistanceSensor.class, "blueSensor");

        return new IntakeHardware(
                slideMotorSettings.makeExMotor(hardwareMap),
                sweepMotorSettings.makeExMotor(hardwareMap),
                sweepLiftServoSettings.makeServo(hardwareMap),
                robotLiftMotorSettings.makeExMotor(hardwareMap),
                grabberServoSettings.makeServo(hardwareMap),
                lowLiftLimit,
                highLiftLimit,
                lowSlideLimit,
                swingServoLeftSettings.makeServo(hardwareMap),
                swingServoRightSettings.makeServo(hardwareMap),
                launchServoAngleSettings.makeServo(hardwareMap),
                launchServoReleaseSettings.makeServo(hardwareMap),
                botSensor,
                topSensor,
                backSensor,
                grabberLimit,
                redSensor,
                blueSensor
        );
    }
}