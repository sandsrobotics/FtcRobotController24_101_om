package org.firstinspires.ftc.teamcode.parts.dropper.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class DropperHardware {

    public static final double slideHoldPower = 1;

    public final DcMotorEx sliderMotor;
    public final Servo grabberServo;
    public final DigitalChannel slideLowLimitSwitch;
    public final Servo swingServoLeft;
    public final Servo swingServoRight;
    public final Rev2mDistanceSensor backSensor;
    public final DigitalChannel grabberLimitSwitch;


    public DropperHardware(DcMotorEx sliderMotor, Servo grabberServo,
                           DigitalChannel slideLowLimitSwitch, Servo swingServoLeft,Servo swingServoRight,
                           Rev2mDistanceSensor backSensor, DigitalChannel grabberLimitSwitch) {
        this.sliderMotor = sliderMotor;
        this.grabberServo = grabberServo;
        this.slideLowLimitSwitch = slideLowLimitSwitch;
        this.swingServoLeft = swingServoLeft;
        this.swingServoRight = swingServoRight;
        this.backSensor = backSensor;
        this.grabberLimitSwitch = grabberLimitSwitch;
    }

    public static DropperHardware makeDefault(HardwareMap hardwareMap){
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
        ServoSettings grabberServoSettings = new ServoSettings(ServoSettings.Number.ZERO, Servo.Direction.FORWARD);
        ServoSettings swingServoLeftSettings = new ServoSettings(ServoSettings.Number.ONE, Servo.Direction.FORWARD);
        ServoSettings swingServoRightSettings = new ServoSettings(ServoSettings.Number.TWO, Servo.Direction.REVERSE);
        Rev2mDistanceSensor backSensor = hardwareMap.get(Rev2mDistanceSensor.class, "backSensor");
        DigitalChannel lowSlideLimit = hardwareMap.get(DigitalChannel.class, "digital3");
        DigitalChannel grabberLimit = hardwareMap.get(DigitalChannel.class, "digital6");
        lowSlideLimit.setMode(DigitalChannel.Mode.INPUT);
        grabberLimit.setMode(DigitalChannel.Mode.INPUT);


        return new DropperHardware(
                slideMotorSettings.makeExMotor(hardwareMap),
                grabberServoSettings.makeServo(hardwareMap),
                lowSlideLimit,
                swingServoLeftSettings.makeServo(hardwareMap),
                swingServoRightSettings.makeServo(hardwareMap),
                backSensor,
                grabberLimit
        );
    }}
