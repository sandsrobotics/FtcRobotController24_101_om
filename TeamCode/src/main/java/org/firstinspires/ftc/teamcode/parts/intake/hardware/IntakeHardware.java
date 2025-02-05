package org.firstinspires.ftc.teamcode.parts.intake.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.motor.MotorSettings;

public class IntakeHardware {
    public static final double slideHoldPower = 1;
    public static final double liftHoldPower = 1;
    public final DcMotorEx liftMotor;
    public final DcMotorEx slideMotor;
    public final DcMotorEx robotHangMotor;
    public final ServoSSR spinner;
    public final ServoSSR flipper;
    public final ServoSSR chute;
    public final ServoSSR pinch;
    public final ServoSSR park;
    public final NormalizedColorSensor colorSensor;
    public final Rev2mDistanceSensor distanceSensor;

    public final DigitalChannel liftZeroSwitch;
    public final DigitalChannel slideZeroSwitch;

    public IntakeHardware(DcMotorEx liftMotor, DcMotorEx slideMotor, DcMotorEx robotHangMotor, DigitalChannel liftZeroSwitch, DigitalChannel slideZeroSwitch,
                          ServoSSR spinner, ServoSSR flipper, ServoSSR chute, ServoSSR pinch, ServoSSR park,
                          NormalizedColorSensor colorSensor, Rev2mDistanceSensor distanceSensor) {
        this.liftMotor = liftMotor;
        this.slideMotor = slideMotor;
        this.robotHangMotor = robotHangMotor;
        this.liftZeroSwitch = liftZeroSwitch;
        this.slideZeroSwitch = slideZeroSwitch;
        this.spinner = spinner;
        this.flipper = flipper;
        this.chute = chute;
        this.pinch = pinch;
        this.park = park;
        this.colorSensor = colorSensor;
        this.distanceSensor = distanceSensor;

        DcMotorEx[] motors = {this.liftMotor, this.robotHangMotor};  //this.slideMotor is plenty fast
        for(DcMotorEx motor : motors){
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }
//beans
    public static IntakeHardware makeDefault(HardwareMap hardwareMap)  {
        MotorSettings liftMotorSettings =new MotorSettings(MotorSettings.Number.ONE_B, DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, liftHoldPower);
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, slideHoldPower);
        MotorSettings robotHangMotorSettings = new MotorSettings(MotorSettings.Number.TWO_B, DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_TO_POSITION, 1);
        DigitalChannel bucketLiftZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital2");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);
        DigitalChannel slideZeroSwitch = hardwareMap.get(DigitalChannel.class, "digital0");
        bucketLiftZeroSwitch.setMode(DigitalChannel.Mode.INPUT);

        return new IntakeHardware(
                liftMotorSettings.makeExMotor(hardwareMap),
                slideMotorSettings.makeExMotor(hardwareMap),
                robotHangMotorSettings.makeExMotor(hardwareMap),
                bucketLiftZeroSwitch,
                slideZeroSwitch,
                new ServoSSR(hardwareMap.get(Servo.class,"servo0")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo2")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo4")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo1")),
                new ServoSSR(hardwareMap.get(Servo.class, "servo0B")),
                hardwareMap.get(NormalizedColorSensor.class, "color"),
                hardwareMap.get(Rev2mDistanceSensor.class, "distance")
        );
    }
}