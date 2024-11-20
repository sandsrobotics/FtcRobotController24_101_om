package org.firstinspires.ftc.teamcode.parts.intake.hardware;

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
    public final DcMotorEx bucketLiftMotor;
    public final DcMotorEx sliderMotor;
    public final Servo tiltServoLeft;
    public final Servo tiltServoRight;
    public final CRServo intakeWheelServoLeft;
    public final CRServo intakeWheelServoRight;
    public final DigitalChannel liftZeroSwitch;

    public IntakeHardware(DcMotorEx bucketLiftMotor, DcMotorEx slideMotor,
                          Servo tiltServoLeft, Servo tiltServoRight, CRServo intakeWheelServoLeft,
                          CRServo intakeWheelServoRight, DigitalChannel liftZeroSwitch) {
        this.bucketLiftMotor = bucketLiftMotor;
        this.sliderMotor = slideMotor;
        this.tiltServoLeft = tiltServoLeft;
        this.tiltServoRight = tiltServoRight;
        this.intakeWheelServoLeft = intakeWheelServoLeft;
        this.intakeWheelServoRight = intakeWheelServoRight;
        this.liftZeroSwitch = liftZeroSwitch;
    }
//beans
    public static IntakeHardware makeDefault(HardwareMap hardwareMap) {
        MotorSettings bucketLiftMotorSettings =new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, bucketHoldPower);
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
        ServoSettings tiltServoLeftSettings = new ServoSettings(ServoSettings.Number.FOUR, Servo.Direction.FORWARD);
        ServoSettings tiltServoRightSettings = new ServoSettings(ServoSettings.Number.FIVE, Servo.Direction.FORWARD);
        MotorSettings intakeWheelServoLeftSettings = new MotorSettings(ServoSettings.Number.ZERO, DcMotorSimple.Direction.FORWARD);
        MotorSettings intakeWheelServoRightSettings = new MotorSettings(ServoSettings.Number.TWO, DcMotorSimple.Direction.REVERSE);
        DigitalChannel liftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital0");
        liftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);

        return new IntakeHardware(
                bucketLiftMotorSettings.makeExMotor(hardwareMap),
                slideMotorSettings.makeExMotor(hardwareMap),
                tiltServoLeftSettings.makeServo(hardwareMap),
                tiltServoRightSettings.makeServo(hardwareMap),
                intakeWheelServoLeftSettings.makeCRServo(hardwareMap),
                intakeWheelServoRightSettings.makeCRServo(hardwareMap),
                liftZeroSwitch
        );
    }
}