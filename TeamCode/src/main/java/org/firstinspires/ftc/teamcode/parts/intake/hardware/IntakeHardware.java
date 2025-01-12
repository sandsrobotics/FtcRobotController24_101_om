package org.firstinspires.ftc.teamcode.parts.intake.hardware;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware {
    public static final double slideHoldPower = 1;
    public static final double bucketHoldPower = 1;
    public final DcMotorEx v_SlideMotor;
    public final DcMotorEx horizSliderMotor;
    public final Servo tiltServo;
    public final CRServo intakeFlipperServo;
    public final DigitalChannel bucketLiftZeroSwitch;
    public final Servo specimanClawServo;
    public final Servo intakeAngleServo;


    public IntakeHardware(DcMotorEx v_SlideMotor, DcMotorEx slideMotor, Servo tiltServo,
                          CRServo intakeFlipperServo, DigitalChannel bucketLiftZeroSwitch, Servo specimanClaw,
                          Servo intakeAngleServo) {
        this.v_SlideMotor = v_SlideMotor;
        this.horizSliderMotor = slideMotor;
        this.tiltServo = tiltServo;
        this.intakeFlipperServo = intakeFlipperServo;
        this.bucketLiftZeroSwitch = bucketLiftZeroSwitch;
        this.specimanClawServo = specimanClaw;
        this.intakeAngleServo = intakeAngleServo;
    }
//beans
    public static IntakeHardware makeDefault(HardwareMap hardwareMap)  {
        MotorSettings v_SlideMotorSettings =new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, bucketHoldPower);
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
        ServoSettings tiltServoSettings = new ServoSettings(ServoSettings.Number.FOUR, Servo.Direction.FORWARD);ServoSettings tiltServoRightSettings = new ServoSettings(ServoSettings.Number.FIVE, Servo.Direction.FORWARD);
        MotorSettings intakeFlipperServoSettings = new MotorSettings(ServoSettings.Number.ZERO, DcMotorSimple.Direction.FORWARD);MotorSettings intakeWheelServoRightSettings = new MotorSettings(ServoSettings.Number.TWO, DcMotorSimple.Direction.REVERSE);
        DigitalChannel bucketLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital2");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);
        ServoSettings specimanClawServoSettings = new ServoSettings(ServoSettings.Number.ONE, Servo.Direction.FORWARD);
        ServoSettings intakeAngleServoSettings = new ServoSettings(ServoSettings.Number.TWO, Servo.Direction.FORWARD);


        return new IntakeHardware(
                v_SlideMotorSettings.makeExMotor(hardwareMap),
                slideMotorSettings.makeExMotor(hardwareMap),
                tiltServoSettings.makeServo(hardwareMap),
                intakeFlipperServoSettings.makeCRServo(hardwareMap),
                bucketLiftZeroSwitch,
                specimanClawServoSettings.makeServo(hardwareMap),
                intakeAngleServoSettings.makeServo(hardwareMap)
        );
    }
}