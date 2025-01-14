package org.firstinspires.ftc.teamcode.parts.intake.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware {
    public static final double slideHoldPower = 1;
    public static final double bucketHoldPower = 1;
    public final DcMotorEx bucketLiftMotor;
    public final DcMotorEx horizSliderMotor;
    public final DcMotorEx v_SlideMotor;
    public final ServoSSR spinner;
    public final ServoSSR flipper;
    public final ServoSSR chute;
    public final ServoSSR pinch;
    public final DigitalChannel bucketLiftZeroSwitch;


    public IntakeHardware(DcMotorEx bucketLiftMotor, DcMotorEx v_SlideMotor, DcMotorEx slideMotor, DigitalChannel bucketLiftZeroSwitch, ServoSSR spinner,
                          ServoSSR flipper, ServoSSR chute, ServoSSR pinch) {
        this.bucketLiftMotor = bucketLiftMotor;
        this.v_SlideMotor = v_SlideMotor;
        this.horizSliderMotor = slideMotor;
        this.bucketLiftZeroSwitch = bucketLiftZeroSwitch;
        this.spinner = spinner;
        this.flipper = flipper;
        this.chute = chute;
        this.pinch = pinch;
    }
//beans
    public static IntakeHardware makeDefault(HardwareMap hardwareMap)  {
        MotorSettings bucketLiftMotorSettings =new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, bucketHoldPower);
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
        MotorSettings v_SlideMotorSettings =new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, bucketHoldPower);
        DigitalChannel bucketLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital2");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);

        return new IntakeHardware(
                bucketLiftMotorSettings.makeExMotor(hardwareMap),
                v_SlideMotorSettings.makeExMotor(hardwareMap),
                slideMotorSettings.makeExMotor(hardwareMap),
                bucketLiftZeroSwitch,
                new ServoSSR(hardwareMap.get(Servo.class,"servo0")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo2")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo4")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo1"))
        );
    }
}