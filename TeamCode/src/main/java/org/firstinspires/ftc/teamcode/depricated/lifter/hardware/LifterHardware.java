package org.firstinspires.ftc.teamcode.depricated.lifter.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.DFR304Range;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class LifterHardware {
    public final DcMotor leftLiftMotor;
    public static final double liftHoldPower = 1;//TODO make not static
    public final Servo rightTurnServo;
    public final Servo grabServo;
    //public final ColorRangeSensor leftRange;
    //public final ColorRangeSensor rightRange;
    //public final DistanceSensor leftDistance;
    //public final DistanceSensor rightDistance;
    public final DFR304Range leftUltrasonic;
    public final DFR304Range rightUltrasonic;
    public final DFR304Range midUltrasonic;
    public final RevBlinkinLedDriver blinkin;
    public final DigitalChannel limitSwitch;
    public final RevColorSensorV3 coneSensor;
    //public final DcMotor rightLiftMotor;
    public final Servo leftTurnServo;

    public LifterHardware(DcMotor leftLiftMotor, DcMotor rightLiftMotor, Servo leftTurnServo,
                          Servo rightTurnServo, Servo grabServo, ColorRangeSensor leftRange,
                          ColorRangeSensor rightRange, DistanceSensor leftDistance,
                          DistanceSensor rightDistance, DFR304Range leftUltrasonic,
                          DFR304Range rightUltrasonic, DFR304Range midUltrasonic,
                          RevBlinkinLedDriver blinkin, DigitalChannel limitSwitch,
                          RevColorSensorV3 coneSensor) {
        this.leftLiftMotor = leftLiftMotor;
        //this.rightLiftMotor = rightLiftMotor;
        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.grabServo = grabServo;
        //this.leftRange = leftRange;
        //this.rightRange = rightRange;
        //this.leftDistance = leftDistance;
        //this.rightDistance = rightDistance;
        this.leftUltrasonic = leftUltrasonic;
        this.rightUltrasonic = rightUltrasonic;
        this.midUltrasonic = midUltrasonic;
        this.blinkin = blinkin;
        this.limitSwitch = limitSwitch;
        this.coneSensor = coneSensor;
    }

    public static LifterHardware makeDefault(HardwareMap hardwareMap){
        MotorSettings leftMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_TO_POSITION, liftHoldPower);
        MotorSettings rightMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_TO_POSITION, liftHoldPower);

        ServoSettings leftServoSettings = new ServoSettings(ServoSettings.Number.ZERO_B, Servo.Direction.FORWARD);
        ServoSettings rightServoSettings = new ServoSettings(ServoSettings.Number.TWO_B, Servo.Direction.REVERSE);
        ServoSettings grabServoSettings = new ServoSettings(ServoSettings.Number.FOUR_B, Servo.Direction.FORWARD);
        //ColorRangeSensor leftRange = hardwareMap.get(ColorRangeSensor.class, "range2");
        //ColorRangeSensor rightRange = hardwareMap.get(ColorRangeSensor.class, "range1");
        //DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "blueWallSensor");
        //DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "redWallSensor");
        DFR304Range rightUltraDist = hardwareMap.get(DFR304Range.class, "rightUltraSensor");
        DFR304Range leftUltraDist = hardwareMap.get(DFR304Range.class, "leftUltraSensor");
        DFR304Range midUltraDist = hardwareMap.get(DFR304Range.class, "middleUltraSensor");
        RevBlinkinLedDriver blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"blinkin");
        DFR304Range.Parameters parameters = new DFR304Range.Parameters();
        parameters.maxRange = DFR304Range.MaxRange.CM150;
        parameters.measureMode = DFR304Range.MeasureMode.PASSIVE;
        leftUltraDist.initialize(parameters);
        rightUltraDist.initialize(parameters);
        midUltraDist.initialize(parameters);

        DigitalChannel limit = hardwareMap.get(DigitalChannel.class, "digital0B");
        limit.setMode(DigitalChannel.Mode.INPUT);

        RevColorSensorV3 cone = hardwareMap.get(RevColorSensorV3.class, "cone dist");

        return new LifterHardware(
                leftMotorSettings.makeMotor(hardwareMap),
                rightMotorSettings.makeMotor(hardwareMap),
                leftServoSettings.makeServo(hardwareMap),
                rightServoSettings.makeServo(hardwareMap),
                grabServoSettings.makeServo(hardwareMap),
                //leftGrabServoSettings.makeCRServo(hardwareMap),
                //rightGrabServoSettings.makeCRServo(hardwareMap),
                null,//leftRange,
                null,//rightRange,
                null,//leftDistance,
                null,//rightDistance,
                leftUltraDist,//leftUltraDist
                rightUltraDist, //rightUltraDist
                midUltraDist,
                blinkin,
                limit,
                cone
        );
    }
}
